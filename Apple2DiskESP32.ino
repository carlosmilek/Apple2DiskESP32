#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_task_wdt.h"

// ============================================================
// CREDENCIAIS WI-FI (Modo Station)
// ============================================================
const char* ssid = "SSID";
const char* password = "Senha_WIFI";

WebServer server(80);

// ============================================================
// CONTROLE DE TROCA DE DISCOS
// ============================================================
volatile bool flagTrocaDisco = false;
char novoDiscoPath[64] = "/dos.nib"; 

// ============================================================
// PINOUT E CONFIGURAÇÕES DE DISCO
// ============================================================
const int PIN_PHASE0 = 4;
const int PIN_PHASE1 = 5;
const int PIN_PHASE2 = 6;
const int PIN_PHASE3 = 7;
const int PIN_WREQ   = 15;
const int PIN_ENABLE = 16;
const int PIN_RDATA  = 2;   
const int PIN_WPROT  = 8;   
const int PIN_WDATA  = 18;

const int MAX_TRACKS  = 35;
const int NIB_SIZE    = 232960;
const int TRACK_SIZE  = 6656;

uint8_t* diskBuffer;                                          
uint8_t  trackBuffer[TRACK_SIZE] __attribute__((aligned(4))); 

volatile int  ph_track   = 0;                
volatile int  intTrk     = 0;                
volatile int  prevTrk    = 0;
volatile bool motorMoveu = false;            
volatile bool wdtReady   = false;            

volatile uint32_t bytesSent       = 0;
volatile uint32_t voltasCompletas = 0;
volatile uint32_t stepCnt         = 0;       

TaskHandle_t TaskEmulador = NULL;
TaskHandle_t TaskSerial   = NULL;
TaskHandle_t TaskWeb      = NULL;

static const int magnet2Position[16] = {
  -1,  0,  2,  1,     
   4, -1,  3, -1,     
   6,  7, -1, -1,     
   5, -1, -1, -1      
};
static const int position2Direction[8][8] = {
  {  0,  1,  2,  3,  0, -3, -2, -1 },  
  { -1,  0,  1,  2,  3,  0, -3, -2 },  
  { -2, -1,  0,  1,  2,  3,  0, -3 },  
  { -3, -2, -1,  0,  1,  2,  3,  0 },  
  {  0, -3, -2, -1,  0,  1,  2,  3 },  
  {  3,  0, -3, -2, -1,  0,  1,  2 },  
  {  2,  3,  0, -3, -2, -1,  0,  1 },  
  {  1,  2,  3,  0, -3, -2, -1,  0 },  
};

// ============================================================
// GCR 6-and-2 ENCODING  (DSK/PO -> NIB) - REESCRITO (Bugfix ProDOS)
// ============================================================
char   uploadPathStr[64] = "";
size_t uploadSize        = 0;

static const uint8_t gcr62_table[64] = {
  0x96, 0x97, 0x9a, 0x9b, 0x9d, 0x9e, 0x9f, 0xa6,
  0xa7, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb2, 0xb3,
  0xb4, 0xb5, 0xb6, 0xb7, 0xb9, 0xba, 0xbb, 0xbc,
  0xbd, 0xbe, 0xbf, 0xcb, 0xcd, 0xce, 0xcf, 0xd3,
  0xd6, 0xd7, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde,
  0xdf, 0xe5, 0xe6, 0xe7, 0xe9, 0xea, 0xeb, 0xec,
  0xed, 0xee, 0xef, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
  0xf7, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};

// Matriz universal de interleaving fisico (Standard Apple II Skew)
// Incrivelmente, esta matriz tambem mapeia Setor Logico -> Chunk do ProDOS (.po)
static const int phys_nib[16] = { 0,13,11,9,7,5,3,1,14,12,10,8,6,4,2,15 };

static uint8_t  primary64_buf[256];
static uint8_t  secondary_buf[86];
static uint8_t  cnv_sec[256];
static uint8_t  cnv_gcr[343];
static uint8_t  cnv_addr[2];

static void odd_enc(uint8_t* o, uint8_t v) {
  o[0] = ((v >> 1) & 0x55) | 0xAA;
  o[1] = (v & 0x55) | 0xAA;
}

static void nibbilize(const uint8_t* src, uint8_t* dst) {
  int i, idx, sec;
  uint8_t p;
  memset(primary64_buf, 0, 256);
  memset(secondary_buf, 0, 86);
  for (i = 0; i < 256; i++) {
    primary64_buf[i] = src[i] >> 2;
    idx = i % 86; sec = i / 86;
    p = ((src[i] & 2) >> 1) | ((src[i] & 1) << 1);
    secondary_buf[idx] |= p << (sec * 2);
  }
  int o = 0;
  dst[o++] = gcr62_table[secondary_buf[0] & 0x3F];
  for (i = 1; i < 86; i++)
    dst[o++] = gcr62_table[(secondary_buf[i] ^ secondary_buf[i - 1]) & 0x3F];
  dst[o++] = gcr62_table[(primary64_buf[0] ^ secondary_buf[85]) & 0x3F];
  for (i = 1; i < 256; i++)
    dst[o++] = gcr62_table[(primary64_buf[i] ^ primary64_buf[i - 1]) & 0x3F];
  dst[o] = gcr62_table[primary64_buf[255] & 0x3F];
}

static bool convert_dsk_to_nib(const char* in_path, const char* nib_path) {
  File dsk = LittleFS.open(in_path, "r");
  if (!dsk || dsk.size() != 143360) { if(dsk) dsk.close(); return false; }
  
  bool is_po = false;
  String pathStr = String(in_path);
  pathStr.toLowerCase();
  if (pathStr.endsWith(".po")) {
    is_po = true;
  }
  
  const uint8_t AP[]={0xD5,0xAA,0x96}, AE[]={0xDE,0xAA,0xEB};
  const uint8_t DP[]={0xD5,0xAA,0xAD}, DE[]={0xDE,0xAA,0xEB};
  memset(diskBuffer, 0xFF, NIB_SIZE); 
  
  for (int trk = 0; trk < 35; trk++) {
    uint8_t* nib = &diskBuffer[trk * TRACK_SIZE];
    
    // O loop agora corre por Setores Logicos (0 a 15)
    for (int L = 0; L < 16; L++) {
      // Chunk no arquivo: 
      // Em .dsk o arquivo esta em ordem logica pura (L)
      // Em .po o arquivo esta em blocos (a conversao eh o proprio phys_nib)
      int file_chunk = is_po ? phys_nib[L] : L;
      
      // Posicao Fisica na trilha para colocar os bytes GCR
      int pi = phys_nib[L];
      
      dsk.seek(trk * 4096 + file_chunk * 256);
      dsk.read(cnv_sec, 256);
      nibbilize(cnv_sec, cnv_gcr);
      
      uint8_t* buf = &nib[pi * 416];
      memcpy(buf + 48, AP, 3);  
      odd_enc(cnv_addr, 0xFE); memcpy(buf + 51, cnv_addr, 2); 
      odd_enc(cnv_addr, trk);  memcpy(buf + 53, cnv_addr, 2); 
      odd_enc(cnv_addr, L);    memcpy(buf + 55, cnv_addr, 2); // Setor Logico L no Header!
      odd_enc(cnv_addr, (uint8_t)(0xFE ^ trk ^ L)); memcpy(buf + 57, cnv_addr, 2); 
      memcpy(buf + 59, AE, 3);  
      memset(buf + 62, 0xFF, 5); 
      memcpy(buf + 67, DP, 3);  
      memcpy(buf + 70, cnv_gcr, 343); 
      memcpy(buf + 413, DE, 3);
    }
  }
  dsk.close();
  File nibFile = LittleFS.open(nib_path, "w");
  if (!nibFile) return false;
  nibFile.write(diskBuffer, NIB_SIZE); nibFile.close();
  return true;
}

// ============================================================
// INTERRUPÇÃO DO MOTOR
// ============================================================
static void IRAM_ATTR enable_isr() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskEmulador, &woken);
  if (woken == pdTRUE) portYIELD_FROM_ISR();
}

// ============================================================
// EMULATOR TASK (Core 1)
// ============================================================
void codigoCore1(void * pvParameters) {
  int      bytePtr  = 0;
  int      bitPos   = 0;
  uint8_t  rByte    = trackBuffer[0];

  int      phase_lastPos = 0;
  bool     phase_ready   = false;

  const uint32_t CPB  = 960;                 
  const uint32_t PUL  = 240;                 

  for (;;) {
    uint32_t gi = REG_READ(GPIO_IN_REG);

    if ((gi & (1 << PIN_ENABLE)) == 0) { 

      pinMode(PIN_RDATA, OUTPUT);            

      if ((gi & (1 << PIN_WREQ)) != 0) {

        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_RDATA); 

        portDISABLE_INTERRUPTS(); 
        uint32_t next = ESP.getCycleCount();

        for (;;) {
          while ((int32_t)(ESP.getCycleCount() - next) < 0) {}
          next += CPB;

          if (rByte & 0x80) {
            REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_RDATA); 
            uint32_t pStart = ESP.getCycleCount();
            while ((int32_t)(ESP.getCycleCount() - pStart - PUL) < 0) {}
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_RDATA); 
          }

          rByte <<= 1;
          
          if (++bitPos >= 8) {
            bitPos = 0;
            bytesSent++;
            
            if (rByte == 0xFF) {
              portENABLE_INTERRUPTS(); 
              __asm__ __volatile__("nop; nop; nop; nop;");
              portDISABLE_INTERRUPTS();
              next = ESP.getCycleCount();
            }

            uint32_t current_gpio = REG_READ(GPIO_IN_REG);
            int f0 = (current_gpio >> PIN_PHASE3) & 1;
            int f1 = (current_gpio >> PIN_PHASE2) & 1;
            int f2 = (current_gpio >> PIN_PHASE1) & 1;
            int f3 = (current_gpio >> PIN_PHASE0) & 1;
            
            int stp = f3 | (f2 << 1) | (f1 << 2) | (f0 << 3);
            int newPos = magnet2Position[stp];

            if (newPos >= 0) {
              if (!phase_ready) {
                phase_lastPos = newPos;
                phase_ready = true;
              } else {
                int move = position2Direction[phase_lastPos][newPos];
                if (move != 0) {
                  ph_track += move; 
                  
                  if (ph_track < 0) ph_track = 0;
                  if (ph_track > 139) ph_track = 139;
                  phase_lastPos = newPos;
                  stepCnt++;
                  
                  int newTrk = ph_track >> 2;
                  if (newTrk != intTrk) {
                    intTrk = newTrk;
                    motorMoveu = true;
                    break; 
                  }
                }
              }
            }

            if (((current_gpio & ((1 << PIN_ENABLE) | (1 << PIN_WREQ))) != (1 << PIN_WREQ))) {
              break; 
            }

            bytePtr++;
            if (bytePtr >= TRACK_SIZE) {
              bytePtr = 0;
              voltasCompletas++;
              rByte = trackBuffer[0];
              break; 
            } else {
              rByte = trackBuffer[bytePtr];
            }
          }
        }
        
        portENABLE_INTERRUPTS(); 
        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_RDATA); 

      } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_RDATA);   
        vTaskDelay(1); 
      }

      if (motorMoveu) {
        motorMoveu = false;
        if (intTrk != prevTrk && intTrk >= 0 && intTrk < MAX_TRACKS) {
          memcpy(trackBuffer, &diskBuffer[intTrk * TRACK_SIZE], TRACK_SIZE);
          prevTrk = intTrk;
          bytePtr = 0; bitPos = 0;
          rByte   = trackBuffer[0];
        }
      }

    } else {
      pinMode(PIN_RDATA, OUTPUT);                
      digitalWrite(PIN_RDATA, LOW);                  
      phase_ready = false;                           
      
      if (flagTrocaDisco) {
        flagTrocaDisco = false;
        File f = LittleFS.open(novoDiscoPath, "r");
        if (f) {
          f.read(diskBuffer, NIB_SIZE);
          f.close();
          intTrk = ph_track >> 2;
          memcpy(trackBuffer, &diskBuffer[intTrk * TRACK_SIZE], TRACK_SIZE);
          prevTrk = intTrk;
          bytePtr = 0; bitPos = 0;
          rByte = trackBuffer[0];
          Serial.printf("\n[SISTEMA] Novo disco no Drive: %s\n", novoDiscoPath);
        } else {
          Serial.printf("\n[SISTEMA] ERRO: Nao foi possivel ler %s\n", novoDiscoPath);
        }
      }

      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
  }
}

// ============================================================
// INTERFACE HTML - TEMA "GREEN PHOSPHOR CRT"
// ============================================================
const char* htmlUI = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Apple II Disk Controller</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=VT323&display=swap');
    
    body { 
      background-color: #050505; color: #33ff33; 
      font-family: 'VT323', 'Courier New', Courier, monospace; 
      font-size: 22px; padding: 20px; 
      text-shadow: 0 0 5px #33ff33; margin: 0;
    }
    .scanlines {
      position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: linear-gradient(to bottom, rgba(255,255,255,0), rgba(255,255,255,0) 50%, rgba(0,0,0,0.2) 50%, rgba(0,0,0,0.2));
      background-size: 100% 4px; pointer-events: none; z-index: 9999;
    }
    h2, h3 { 
      border-bottom: 2px solid #33ff33; padding-bottom: 5px; 
      text-transform: uppercase; box-shadow: 0 4px 4px -4px #33ff33;
    }
    button, input[type="submit"], input[type="file"] { 
      background-color: #050505; color: #33ff33; border: 1px solid #33ff33; 
      padding: 8px 15px; margin: 5px 0; cursor: pointer; font-family: inherit; 
      font-size: 20px; text-transform: uppercase; box-shadow: 0 0 5px #33ff33;
      transition: all 0.2s;
    }
    button:hover, input[type="submit"]:hover { background-color: #33ff33; color: #050505; }
    .btn-danger { border-color: #ff3333; color: #ff3333; box-shadow: 0 0 5px #ff3333; text-shadow: none; }
    .btn-danger:hover { background-color: #ff3333; color: #050505; }
    .disk-item { 
      margin: 10px 0; padding: 10px; border: 1px dashed #33ff33; 
      display: flex; justify-content: space-between; align-items: center; 
      background: rgba(51, 255, 51, 0.05);
    }
    .storage-container { margin-bottom: 25px; }
    .storage-text { margin-bottom: 5px; font-weight: normal; letter-spacing: 1px; }
    .storage-box {
      border: 2px solid #33ff33; width: 100%; height: 25px; 
      background-color: #000; box-shadow: 0 0 8px #33ff33; position: relative;
    }
    .storage-bar {
      height: 100%; background-color: #33ff33; 
      box-shadow: 0 0 10px #33ff33; transition: width 0.5s;
    }
  </style>
</head>
<body>
  <div class="scanlines"></div>
  <h2>DISK II CONTROLLER - ESP32-S3</h2>
  
  <div class="storage-container">
    <div class="storage-text" id="infoText">STORAGE: CALCULATING...</div>
    <div class="storage-box"><div class="storage-bar" id="storageBar" style="width: 0%;"></div></div>
  </div>

  <h3>AVAILABLE DISKS</h3>
  <div id="listaDiscos">Loading...</div>
  <br>

  <h3>UPLOAD NEW DISK (.dsk, .po, .nib)</h3>
  <form method="POST" action="/upload" enctype="multipart/form-data">
    <input type="file" name="f" accept=".nib,.dsk,.po"><br><br>
    <input type="submit" value="SEND TO ESP32">
  </form>

  <script>
    function carregarDados() {
      fetch('/list').then(response => response.text()).then(html => {
        document.getElementById('listaDiscos').innerHTML = html;
      });
      fetch('/storage').then(response => response.json()).then(data => {
        let pct = (data.used / data.total) * 100;
        document.getElementById('storageBar').style.width = pct + '%';
        document.getElementById('infoText').innerText = 
          'LITTLEFS STORAGE: ' + Math.round(data.used/1024) + 'KB / ' + Math.round(data.total/1024) + 'KB (' + Math.round(pct) + '%)';
      });
    }
    
    function montarDisco(nomeArquivo) {
      fetch('/mount?file=' + encodeURIComponent(nomeArquivo)).then(response => {
        alert('COMMAND ACCEPTED. DISK WILL MOUNT.');
      });
    }

    function apagarDisco(nomeArquivo) {
      if(confirm('DELETE ' + nomeArquivo + '?')) {
        fetch('/delete?file=' + encodeURIComponent(nomeArquivo)).then(response => {
          carregarDados(); 
        });
      }
    }
    
    window.onload = carregarDados;
  </script>
</body>
</html>
)rawliteral";

// ============================================================
// WEB SERVER TASK E ROTAS (Core 0)
// ============================================================
File fsUploadFile;

void tarefaWeb(void * pvParameters) {
  
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlUI);
  });

  server.on("/storage", HTTP_GET, []() {
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    String res = "{\"total\":" + String(total) + ",\"used\":" + String(used) + "}";
    server.send(200, "application/json", res);
  });

  server.on("/list", HTTP_GET, []() {
    String htmlLista = "";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){
      String fileName = String(file.name());
      if (fileName.endsWith(".nib") || fileName.endsWith(".dsk") || fileName.endsWith(".po")) {
        htmlLista += "<div class='disk-item'>";
        htmlLista += "<span><strong>" + fileName + "</strong> (" + String(file.size() / 1024) + " KB)</span>";
        htmlLista += "<div><button onclick=\"montarDisco('" + fileName + "')\">MOUNT</button> ";
        htmlLista += "<button class='btn-danger' onclick=\"apagarDisco('" + fileName + "')\">DEL</button></div>";
        htmlLista += "</div>";
      }
      file = root.openNextFile();
    }
    if(htmlLista == "") htmlLista = "NO DISKS FOUND.";
    server.send(200, "text/html", htmlLista);
  });

  server.on("/delete", HTTP_GET, []() {
    if(server.hasArg("file")) {
      String fileName = server.arg("file");
      if(!fileName.startsWith("/")) fileName = "/" + fileName;
      
      if(LittleFS.remove(fileName)) {
        server.send(200, "text/plain", "FILE DELETED");
      } else {
        server.send(500, "text/plain", "ERROR DELETING");
      }
    } else {
      server.send(400, "text/plain", "FILE ARGUMENT MISSING");
    }
  });

  server.on("/mount", HTTP_GET, []() {
    if(server.hasArg("file")) {
      String fileName = server.arg("file");
      if(!fileName.startsWith("/")) fileName = "/" + fileName;
      
      fileName.toCharArray(novoDiscoPath, sizeof(novoDiscoPath));
      flagTrocaDisco = true;
      
      server.send(200, "text/plain", "COMMAND RECEIVED");
    } else {
      server.send(400, "text/plain", "FILE ARGUMENT MISSING");
    }
  });

  server.on("/upload", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    
    // UI Atualizada do Fósforo Verde para a página de retorno de Upload
    String upHtml = R"rawliteral(
    <!DOCTYPE html><html><head><meta charset="UTF-8"><style>
      @import url('https://fonts.googleapis.com/css2?family=VT323&display=swap');
      body { background-color: #050505; color: #33ff33; font-family: 'VT323', monospace; font-size: 26px; text-shadow: 0 0 5px #33ff33; margin: 0; display: flex; flex-direction: column; justify-content: center; align-items: center; height: 100vh; }
      .scanlines { position: fixed; top: 0; left: 0; width: 100vw; height: 100vh; background: linear-gradient(to bottom, rgba(255,255,255,0), rgba(255,255,255,0) 50%, rgba(0,0,0,0.2) 50%, rgba(0,0,0,0.2)); background-size: 100% 4px; pointer-events: none; z-index: 9999; }
      a { color: #050505; background-color: #33ff33; text-decoration: none; border: 2px solid #33ff33; padding: 10px 20px; margin-top: 20px; text-transform: uppercase; box-shadow: 0 0 10px #33ff33; }
      a:hover { background-color: #050505; color: #33ff33; }
    </style></head><body>
    <div class="scanlines"></div>
    <h2>UPLOAD COMPLETE</h2>
    <a href="/">RETURN TO SYSTEM</a>
    </body></html>
    )rawliteral";
    
    server.send(200, "text/html", upHtml);
    
    String upPath = String(uploadPathStr);
    upPath.toLowerCase();
    
    if (upPath.endsWith(".dsk") || upPath.endsWith(".po")) {
      char nibPath[64];
      strcpy(nibPath, uploadPathStr);
      char* d = strrchr(nibPath, '.'); 
      if (d) { 
        strcpy(d, ".nib");
        Serial.printf("[SISTEMA] Iniciando conversao de %s para %s...\n", uploadPathStr, nibPath);
        if (convert_dsk_to_nib(uploadPathStr, nibPath)) {
          LittleFS.remove(uploadPathStr); 
          Serial.println("[SISTEMA] Conversao concluida com sucesso.");
        } else {
          Serial.println("[SISTEMA] Falha na conversao.");
        }
      }
    }
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (!filename.startsWith("/")) filename = "/" + filename;
      strncpy(uploadPathStr, filename.c_str(), 63);
      Serial.printf("[WEB] Iniciando upload: %s\n", filename.c_str());
      fsUploadFile = LittleFS.open(filename, "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (fsUploadFile) fsUploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) fsUploadFile.close();
      Serial.printf("[WEB] Upload concluido: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
    }
  });

  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ============================================================
// SERIAL MONITOR TASK (Core 0)
// ============================================================
void tarefaSerial(void * pvParameters) {
  static int      lastTrack   = -1;
  static uint32_t lastMillis  = 0;
  static uint32_t prevBytes   = 0;

  for (;;) {
    int t = intTrk;
    if (t != lastTrack) {
      Serial.printf("\n>>> TRACK %d (ph:%d) <<<\n", t, ph_track);
      lastTrack = t;
    }

    uint32_t now = millis();
    if (now - lastMillis >= 2000) {
      lastMillis  = now;
      uint32_t bps = (bytesSent - prevBytes) / 2;
      prevBytes  = bytesSent;
      Serial.printf("T:%d ph:%d tx:%u b/s vol:%u passos:%u\n", t, ph_track, bps, (unsigned)voltasCompletas, stepCnt);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  setCpuFrequencyMhz(240);                   

  Serial.println(F("\n========== APPLE II DISK II (ESP32-S3 Wi-Fi) =========="));

  if (!psramFound()) {
    Serial.println(F("FATAL: PSRAM not available"));
    while (1) {}
  }
  
  diskBuffer = (uint8_t*)ps_malloc(NIB_SIZE);
  
  if (!LittleFS.begin(true)) {
    Serial.println(F("FATAL: LittleFS init failed"));
    while (1) {}
  }

  Serial.printf("Conectando na rede: %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Wi-Fi] Conectado!");
  Serial.print("[Wi-Fi] Acesse no navegador: http://");
  Serial.println(WiFi.localIP());
  Serial.println(F("======================================================"));

  File f = LittleFS.open(novoDiscoPath, "r");
  if (f) {
    size_t rd = f.read(diskBuffer, NIB_SIZE);
    f.close();
    Serial.printf("Disco inicial montado: %s (%u bytes)\n", novoDiscoPath, rd);
  } else {
    Serial.println("AVISO: dos.nib nao encontrado. Faca o upload pela pagina web.");
  }
  
  memcpy(trackBuffer, &diskBuffer[0], TRACK_SIZE);
  prevTrk = 0;

  pinMode(PIN_PHASE0, INPUT);
  pinMode(PIN_PHASE1, INPUT);
  pinMode(PIN_PHASE2, INPUT);
  pinMode(PIN_PHASE3, INPUT);
  pinMode(PIN_WREQ,   INPUT);
  pinMode(PIN_ENABLE, INPUT_PULLUP);
  pinMode(PIN_WDATA,  INPUT);

  pinMode(PIN_RDATA, OUTPUT);
  digitalWrite(PIN_RDATA, LOW);    

  pinMode(PIN_WPROT, OUTPUT);
  digitalWrite(PIN_WPROT, HIGH);    

  xTaskCreatePinnedToCore(codigoCore1, "Emu", 16384, NULL, 10, &TaskEmulador, 1);
  xTaskCreatePinnedToCore(tarefaSerial, "Ser", 4096, NULL, 1, &TaskSerial, 0);
  xTaskCreatePinnedToCore(tarefaWeb, "Web", 8192, NULL, 1, &TaskWeb, 0);

  wdtReady = true;
  attachInterrupt(digitalPinToInterrupt(PIN_ENABLE), enable_isr, FALLING);

  if ((REG_READ(GPIO_IN_REG) & (1 << PIN_ENABLE)) == 0) {
    xTaskNotifyGive(TaskEmulador);
  }
}

void loop() {
  vTaskDelete(NULL);
}
