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
const char* ssid = "Milek";
const char* password = "Milek1234";

WebServer server(80);
String     uploadPath = "";     // current upload file path
size_t     uploadSize  = 0;     // bytes received so far
uint8_t*   tempDskBuf  = NULL;  // temp buffer for DSK→NIB conversion (PSRAM)

// ============================================================
// CONTROLE DE TROCA DE DISCOS (Handshake Core 0 -> Core 1)
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

static const int magnet2Position[16] = {
  -1,  0,  2,  1,     
   4, -1,  3, -1,     
   6,  7, -1, -1,     
   5, -1, -1, -1      
};
static const int position2Direction[8][8] = {
  {  0,  1,  2,  3,  0, -3, -2, -1 },  // N
  { -1,  0,  1,  2,  3,  0, -3, -2 },  // NE
  { -2, -1,  0,  1,  2,  3,  0, -3 },  // E
  { -3, -2, -1,  0,  1,  2,  3,  0 },  // SE
  {  0, -3, -2, -1,  0,  1,  2,  3 },  // S
  {  3,  0, -3, -2, -1,  0,  1,  2 },  // SW
  {  2,  3,  0, -3, -2, -1,  0,  1 },  // W
  {  1,  2,  3,  0, -3, -2, -1,  0 },  // NW
};

// ============================================================
// GCR 6‑and‑2 ENCODING  (DSK → NIB conversion)
// Algorithm exactly matching slotek/dsk2nib reference
// ============================================================
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

// Logical sector → NIB physical position (same for DOS 3.3 and ProDOS 5.25")
static const int phys_nib[16] = { 0,13,11,9,7,5,3,1,14,12,10,8,6,4,2,15 };

// Which DSK file position holds data for logical sector L
// "ProDOS order" DSK (CiderPress default .po/.dsk)
static const int dsk2log_po[16]  = { 0,2,4,6,8,10,12,14,1,3,5,7,9,11,13,15 };
// "DOS 3.3 order" DSK
static const int dsk2log_dos[16] = { 0,7,14,6,13,5,12,4,11,3,10,2,9,1,8,15 };

// Auto-detect: ProDOS boot block byte $23 bit 0
static int detect_dsk_fmt(File& f) {
  uint8_t buf[256];
  f.seek(0);
  f.read(buf, 256);
  return (buf[35] & 0x01) ? 1 : 0;   // 1=ProDOS, 0=DOS 3.3
}

// Address field encoding: 1 byte → 2 "4-and-4" bytes
static void odd_even_encode(uint8_t* out, uint8_t val) {
  out[0] = ((val >> 1) & 0x55) | 0xAA;
  out[1] = (val & 0x55) | 0xAA;
}

// 6+2 GCR nibbilize (matches slotek dsk2nib exactly)
static uint8_t primary64_buf[256];
static uint8_t secondary_buf[86];

static void nibbilize(const uint8_t* src, uint8_t* dest) {
  int i, index, section;
  uint8_t pair;

  memset(primary64_buf, 0, sizeof(primary64_buf));
  memset(secondary_buf, 0, sizeof(secondary_buf));

  for (i = 0; i < 256; i++) {
    primary64_buf[i] = src[i] >> 2;
    index   = i % 86;
    section = i / 86;
    pair = ((src[i] & 2) >> 1) | ((src[i] & 1) << 1);
    secondary_buf[index] |= pair << (section * 2);
  }

  int out = 0;
  dest[out++] = gcr62_table[secondary_buf[0]        & 0x3F];
  for (i = 1; i < 86; i++)
    dest[out++] = gcr62_table[(secondary_buf[i] ^ secondary_buf[i - 1]) & 0x3F];
  dest[out++] = gcr62_table[(primary64_buf[0] ^ secondary_buf[85]) & 0x3F];
  for (i = 1; i < 256; i++)
    dest[out++] = gcr62_table[(primary64_buf[i] ^ primary64_buf[i - 1]) & 0x3F];
  dest[out++] = gcr62_table[primary64_buf[255] & 0x3F];
}

bool convert_dsk_to_nib(const char* dsk_path, const char* nib_path) {
  File dsk = LittleFS.open(dsk_path, "r");
  if (!dsk) { Serial.printf("[DSK] ERRO abrindo %s\n", dsk_path); return false; }
  if (dsk.size() != 143360) { Serial.printf("[DSK] tamanho invalido: %u\n", dsk.size()); dsk.close(); return false; }

  int fmt = detect_dsk_fmt(dsk);
  const int* dsk2log = (fmt == 1) ? dsk2log_po : dsk2log_dos;
  Serial.printf("[CONV] Formato: %s\n", (fmt == 1) ? "ProDOS" : "DOS 3.3");

  const uint8_t ADDR_P[]   = {0xD5, 0xAA, 0x96};
  const uint8_t ADDR_E[]   = {0xDE, 0xAA, 0xEB};
  const uint8_t DATA_P[]   = {0xD5, 0xAA, 0xAD};
  const uint8_t DATA_E[]   = {0xDE, 0xAA, 0xEB};
  const uint8_t VOLUME = 0xFE;

  uint8_t sec_data[256], gcr_out[343];
  uint8_t addr_enc[2];

  memset(diskBuffer, 0xFF, NIB_SIZE);

  for (int trk = 0; trk < 35; trk++) {
    uint8_t* nib = &diskBuffer[trk * TRACK_SIZE];

    for (int sec = 0; sec < 16; sec++) {
      int dsk_idx = dsk2log[sec];                 // which DSK sector index
      int phy_idx = phys_nib[sec];                 // NIB position (same for DOS/ProDOS)

      dsk.seek(trk * 4096 + dsk_idx * 256);
      dsk.read(sec_data, 256);
      nibbilize(sec_data, gcr_out);

      // Build sector at NIB physical position
      uint8_t* buf = &nib[phy_idx * 416];

      // gap1 (already filled with 0xFF by memset above)

      // Address field
      memcpy(buf + 48, ADDR_P, 3);
      odd_even_encode(addr_enc, VOLUME);
      memcpy(buf + 51, addr_enc, 2);
      odd_even_encode(addr_enc, trk);
      memcpy(buf + 53, addr_enc, 2);
      odd_even_encode(addr_enc, sec);              // logical sector number
      memcpy(buf + 55, addr_enc, 2);
      odd_even_encode(addr_enc, VOLUME ^ trk ^ sec);
      memcpy(buf + 57, addr_enc, 2);
      memcpy(buf + 59, ADDR_E, 3);

      // gap2
      memset(buf + 62, 0xFF, 5);

      // Data field
      memcpy(buf + 67, DATA_P, 3);
      memcpy(buf + 70, gcr_out, 343);
      memcpy(buf + 413, DATA_E, 3);
    }
  }
  dsk.close();

  File nib = LittleFS.open(nib_path, "w");
  if (!nib) { Serial.printf("[NIB] ERRO criando %s\n", nib_path); return false; }
  nib.write(diskBuffer, NIB_SIZE);
  nib.close();
  Serial.printf("[CONV] Criado: %s (232960 bytes)\n", nib_path);
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
// EMULATOR TASK (Core 1) - Com "Respiradouro de Sincronia"
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

    if ((gi & (1 << PIN_ENABLE)) == 0) { // MOTOR LIGADO

      pinMode(PIN_RDATA, OUTPUT);            

      // ---- READ MODE (/WREQ = 1) ----
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
            
            // ========================================================
            // RESPIRADOURO DO SISTEMA: Esconde o Wi-Fi no Sync Gap!
            // ========================================================
            if (rByte == 0xFF) {
              portENABLE_INTERRUPTS(); 
              // Dá alguns ciclos para o Wi-Fi ou IPC agir
              __asm__ __volatile__("nop; nop; nop; nop;");
              portDISABLE_INTERRUPTS();
              
              // Ressincroniza o relógio para não acelerar o próximo bit
              next = ESP.getCycleCount();
            }

            // POLLING DO MOTOR
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
        // ---- WRITE MODE (/WREQ = 0) ----
        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_RDATA);   
        vTaskDelay(1); 
      }

      // ---- LOAD TRACK ----
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
      // ---- DRIVE NOT SELECTED (MOTOR DESLIGADO) ----
      pinMode(PIN_RDATA, OUTPUT);                 
      digitalWrite(PIN_RDATA, LOW);                  
      phase_ready = false;                           
      
      // ROTINA DE TROCA DE DISCO SEGURA
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
          Serial.printf("\n[SISTEMA] Novo disco na agulha: %s\n", novoDiscoPath);
        } else {
          Serial.printf("\n[SISTEMA] ERRO: Nao foi possivel ler %s\n", novoDiscoPath);
        }
      }

      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
  }
}

// ============================================================
// INTERFACE HTML (Embutida na memória)
// ============================================================
const char* htmlUI = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Disk II - Apple II</title>
  <style>
    body { background-color: #000; color: #33ff33; font-family: 'Courier New', Courier, monospace; padding: 20px; }
    h2 { border-bottom: 1px solid #33ff33; padding-bottom: 10px; }
    button, input[type="submit"], input[type="file"] { background-color: #33ff33; color: #000; border: none; padding: 8px 14px; margin: 3px 0; cursor: pointer; font-family: inherit; font-weight: bold; }
    button:hover, input[type="submit"]:hover { background-color: #fff; }
    .btn-del { background-color: #cc3300; }
    .btn-del:hover { background-color: #ff4444; }
    .disk-item { margin: 10px 0; padding: 10px; border: 1px dashed #33ff33; display: flex; align-items: center; gap: 10px; }
    .disk-item span { flex: 1; }
    .storage { margin: 10px 0; padding: 8px 14px; border: 1px solid #33ff33; }
    .storage-bar { height: 16px; background: #333; margin-top: 6px; }
    .storage-bar-fill { height: 100%; background: #33ff33; }
  </style>
</head>
<body>
  <h2>ESP32-S3 Disk II Emulator</h2>
  
  <div id="infoStorage" class="storage">Armazenamento: carregando...</div>

  <h3>Discos Disponiveis</h3>
  <div id="listaDiscos">Carregando...</div>
  <br><br>

  <h3>Subir Novo Disco</h3>
  <form method="POST" action="/upload" enctype="multipart/form-data">
    <input type="file" name="f" accept=".nib,.dsk"><br><br>
    <input type="submit" value="Enviar para o ESP32 (.nib ou .dsk)">
  </form>

  <script>
    function carregarTudo() {
      fetch('/list').then(r=>r.text()).then(html => {
        document.getElementById('listaDiscos').innerHTML = html;
      });
      fetch('/storage').then(r=>r.text()).then(html => {
        document.getElementById('infoStorage').innerHTML = html;
      });
    }
    function montarDisco(nome) {
      fetch('/mount?file=' + encodeURIComponent(nome)).then(() => {
        alert('Disco montado: ' + nome);
      });
    }
    function deletarDisco(nome) {
      if (!confirm('Apagar ' + nome + '?')) return;
      fetch('/delete?file=' + encodeURIComponent(nome)).then(() => {
        alert('Apagado: ' + nome);
        carregarTudo();
      });
    }
    window.onload = carregarTudo;
  </script>
</body>
</html>
)rawliteral";

// ============================================================
// WEB SERVER ROTAS (registradas em setup, executadas em loop)
// ============================================================
File fsUploadFile;

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlUI);
  });

  server.on("/list", HTTP_GET, []() {
    String htmlLista = "";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){
      String fileName = String(file.name());
      if (fileName.endsWith(".nib")) {
        htmlLista += "<div class='disk-item'><span><strong>" + fileName + "</strong></span>";
        htmlLista += "<button onclick=\"montarDisco('" + fileName + "')\">Montar</button>";
        htmlLista += "<button class='btn-del' onclick=\"deletarDisco('" + fileName + "')\">Apagar</button></div>";
      }
      file = root.openNextFile();
    }
    if(htmlLista == "") htmlLista = "Nenhum disco encontrado.";
    server.send(200, "text/html", htmlLista);
  });

  server.on("/delete", HTTP_GET, []() {
    if(server.hasArg("file")) {
      String f = server.arg("file");
      if(!f.startsWith("/")) f = "/" + f;
      if(LittleFS.remove(f)) {
        server.send(200, "text/plain", "OK");
        Serial.printf("[WEB] Apagado: %s\n", f.c_str());
      } else {
        server.send(404, "text/plain", "ERRO");
      }
    } else {
      server.send(400, "text/plain", "Faltou o parametro file");
    }
  });

  server.on("/storage", HTTP_GET, []() {
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    size_t free  = total - used;
    int pct = (total > 0) ? (used * 100 / total) : 0;
    char buf[256];
    snprintf(buf, sizeof(buf),
      "Usado: %.1f MB / %.1f MB (%d%%) &nbsp; Livre: %.1f MB"
      "<div class='storage-bar'><div class='storage-bar-fill' style='width:%d%%'></div></div>",
      used / 1048576.0, total / 1048576.0, pct, free / 1048576.0, pct);
    server.send(200, "text/html", buf);
  });

  server.on("/mount", HTTP_GET, []() {
    if(server.hasArg("file")) {
      String fileName = server.arg("file");
      if(!fileName.startsWith("/")) fileName = "/" + fileName;
      fileName.toCharArray(novoDiscoPath, sizeof(novoDiscoPath));
      flagTrocaDisco = true;
      server.send(200, "text/plain", "Comando recebido");
    } else {
      server.send(400, "text/plain", "Faltou o parametro file");
    }
  });

  server.on("/upload", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", "<body style='background:#000; color:#33ff33;'><br><br><center><h2>Upload Completo!</h2><a href='/' style='color:#fff;'>Voltar</a></center></body>");
    if (uploadPath.endsWith(".dsk")) {
      String nibPath = uploadPath;
      nibPath.replace(".dsk", ".nib");
      if (convert_dsk_to_nib(uploadPath.c_str(), nibPath.c_str())) {
        LittleFS.remove(uploadPath);
      }
    }
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      uploadPath = upload.filename;
      if (!uploadPath.startsWith("/")) uploadPath = "/" + uploadPath;
      uploadSize = 0;
      Serial.printf("[WEB] Iniciando upload: %s\n", uploadPath.c_str());
      fsUploadFile = LittleFS.open(uploadPath, "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (fsUploadFile) {
        fsUploadFile.write(upload.buf, upload.currentSize);
        uploadSize += upload.currentSize;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) fsUploadFile.close();
      Serial.printf("[WEB] Upload concluido: %s (%u bytes)\n", uploadPath.c_str(), uploadSize);
    }
  });

  server.begin();
  Serial.println(F("[WEB] Servidor HTTP iniciado"));
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
  // Alocação limpa
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

  // Task Emulador movida para a prioridade 10 (Mais alta que Wi-Fi e Serial, mas menor que IPC interno)
  // Pilha de memória aumentada para 16384 bytes para blindagem
  xTaskCreatePinnedToCore(codigoCore1, "Emu", 16384, NULL, 10, &TaskEmulador, 1);
  xTaskCreatePinnedToCore(tarefaSerial, "Ser", 4096, NULL, 1, &TaskSerial, 0);

  setupWebServer();                                   // register routes + server.begin()

  wdtReady = true;
  attachInterrupt(digitalPinToInterrupt(PIN_ENABLE), enable_isr, FALLING);

  if ((REG_READ(GPIO_IN_REG) & (1 << PIN_ENABLE)) == 0) {
    xTaskNotifyGive(TaskEmulador);
  }
}

void loop() {
  server.handleClient();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
