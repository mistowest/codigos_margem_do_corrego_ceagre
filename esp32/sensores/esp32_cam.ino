/**
 * ESP32-CAM — Streaming MJPEG Otimizado (sem delay, usando millis/esp_timer)
 * Placa: AI-Thinker ESP32-CAM (OV2640)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"

// ─── Configurações ────────────────────────────────────────────────────────────
#define WIFI_SSID     "Ceagre"
#define WIFI_PASSWORD "Ceagre@10"
#define FLASH_LED_PIN 4

// ─── Pinout AI-Thinker ────────────────────────────────────────────────────────
#define CAM_PIN_PWDN  32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK   0
#define CAM_PIN_SIOD  26
#define CAM_PIN_SIOC  27
#define CAM_PIN_D7    35
#define CAM_PIN_D6    34
#define CAM_PIN_D5    39
#define CAM_PIN_D4    36
#define CAM_PIN_D3    21
#define CAM_PIN_D2    19
#define CAM_PIN_D1    18
#define CAM_PIN_D0     5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF  23
#define CAM_PIN_PCLK  22

// ─── Boundary MJPEG ───────────────────────────────────────────────────────────
#define BOUNDARY  "gc0p4Jq0M2Yt08jU534c0p"
#define BOUND_HDR "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: "

// ─── Estado global ────────────────────────────────────────────────────────────
static bool     flashEnabled   = false;
static uint32_t loopLastLog    = 0;       // millis da última linha de log no loop()
static uint32_t wifiReconnectAt = 0;      // millis para tentar reconectar Wi-Fi
static bool     serverStarted  = false;

static httpd_handle_t httpServer = NULL;

// ─── Inicializa câmera ────────────────────────────────────────────────────────
bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel  = LEDC_CHANNEL_0;
  cfg.ledc_timer    = LEDC_TIMER_0;
  cfg.pin_d0  = CAM_PIN_D0;  cfg.pin_d1  = CAM_PIN_D1;
  cfg.pin_d2  = CAM_PIN_D2;  cfg.pin_d3  = CAM_PIN_D3;
  cfg.pin_d4  = CAM_PIN_D4;  cfg.pin_d5  = CAM_PIN_D5;
  cfg.pin_d6  = CAM_PIN_D6;  cfg.pin_d7  = CAM_PIN_D7;
  cfg.pin_xclk     = CAM_PIN_XCLK;
  cfg.pin_pclk     = CAM_PIN_PCLK;
  cfg.pin_vsync    = CAM_PIN_VSYNC;
  cfg.pin_href     = CAM_PIN_HREF;
  cfg.pin_sscb_sda = CAM_PIN_SIOD;
  cfg.pin_sscb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn     = CAM_PIN_PWDN;
  cfg.pin_reset    = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;

  bool psram = psramFound() && ESP.getPsramSize() > 0;
  Serial.printf("[MEM] Heap: %u  PSRAM: %s (%u)\n",
    esp_get_free_heap_size(),
    psram ? "SIM" : "NAO",
    psram ? ESP.getPsramSize() : 0u);

  if (psram) {
    cfg.frame_size   = FRAMESIZE_VGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 15;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK && psram) {
    Serial.println("[WARN] VGA falhou, tentando QVGA DRAM...");
    esp_camera_deinit();
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 15;
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
    cfg.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
    err = esp_camera_init(&cfg);
  }

  if (err != ESP_OK) {
    Serial.printf("[ERRO] Camera: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, GAINCEILING_8X);
  s->set_bpc(s, 0);
  s->set_wpc(s, 0);
  s->set_lenc(s, 0);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);

  Serial.println("[OK] Camera pronta!");
  return true;
}

// ─── Página principal ─────────────────────────────────────────────────────────
static esp_err_t indexHandler(httpd_req_t *req) {
  static const char html[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset=UTF-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP32-CAM</title>"
    "<style>"
    "body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}"
    "h1{padding:14px;margin:0;background:#1a1a1a}"
    "#s{max-width:100%;border:2px solid #333;border-radius:8px;margin:12px auto;display:block}"
    ".b{display:inline-block;margin:5px;padding:9px 20px;border:none;border-radius:6px;"
    "font-size:14px;cursor:pointer;background:#0af;color:#000;font-weight:700}"
    ".b:hover{background:#08c}.r{background:#f44;color:#fff}.r:hover{background:#c22}"
    "select{padding:7px;border-radius:6px;font-size:13px;margin:5px}"
    "</style></head><body>"
    "<h1>ESP32-CAM</h1>"
    "<img id=s src=/stream>"
    "<br>"
    "<button class=b onclick='location=\"/capture\"'>Capturar foto</button>"
    "<button class='b r' id=fb onclick=toggleFlash()>Flash OFF</button><br>"
    "<label>Resolucao: <select onchange='setRes(this.value)'>"
    "<option value=8>SVGA 800x600</option>"
    "<option value=6 selected>VGA 640x480</option>"
    "<option value=5>CIF 352x288</option>"
    "<option value=3>QVGA 320x240</option>"
    "</select></label>"
    "<script>"
    "let f=false;"
    "function toggleFlash(){f=!f;fetch('/flash?s='+(f?1:0));"
    "document.getElementById('fb').textContent='Flash '+(f?'ON':'OFF');}"
    "function setRes(v){fetch('/res?v='+v)"
    ".then(()=>{let s=document.getElementById('s');"
    "s.src='/stream?t='+Date.now();});}"
    "</script></body></html>";

  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, html, sizeof(html) - 1);
}

// ─── MJPEG Stream ─────────────────────────────────────────────────────────────
// O timeout usa esp_timer_get_time() (microsegundos), que não interfere
// com millis() nem com o scheduler do FreeRTOS.
// Não há nenhum vTaskDelay/delay aqui — o loop cede ao RTOS apenas via
// esp_camera_fb_get(), que internamente aguarda o DMA sem bloquear outras tasks.
static esp_err_t streamHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

  char lenBuf[24];
  int64_t lastFrameUs = esp_timer_get_time(); // microsegundos monotônicos
  const int64_t TIMEOUT_US = 5000000LL;       // 5 s em µs

  while (true) {
    // ── Timeout sem delay: compara timestamps µs ──────────────────────────
    if ((esp_timer_get_time() - lastFrameUs) > TIMEOUT_US) {
      Serial.println("[Stream] Timeout do cliente.");
      break;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      // Sem frame disponível: cede 1 tick ao RTOS sem bloquear Wi-Fi/TCP
      taskYIELD();
      continue;
    }

    snprintf(lenBuf, sizeof(lenBuf), "%u\r\n\r\n", (unsigned)fb->len);

    esp_err_t res = httpd_resp_send_chunk(req, BOUND_HDR, sizeof(BOUND_HDR) - 1);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, lenBuf, strlen(lenBuf));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    if (res != ESP_OK) break; // cliente desconectou

    lastFrameUs = esp_timer_get_time(); // reset timeout
    // Sem delay/yield aqui: próximo fb_get() já cede ao RTOS internamente
  }

  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ─── Capturar foto ────────────────────────────────────────────────────────────
static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=foto.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ─── Flash ────────────────────────────────────────────────────────────────────
static esp_err_t flashHandler(httpd_req_t *req) {
  char q[16], v[4];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "s", v, sizeof(v)) == ESP_OK) {
    flashEnabled = (v[0] == '1');
    digitalWrite(FLASH_LED_PIN, flashEnabled ? HIGH : LOW);
  }
  return httpd_resp_send(req, "OK", 2);
}

// ─── Resolução ────────────────────────────────────────────────────────────────
static esp_err_t resHandler(httpd_req_t *req) {
  char q[16], v[4];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "v", v, sizeof(v)) == ESP_OK) {
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_framesize(s, (framesize_t)atoi(v));
  }
  return httpd_resp_send(req, "OK", 2);
}

// ─── Inicia servidor HTTP ─────────────────────────────────────────────────────
void startServer() {
  if (serverStarted) return;

  httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
  cfg.server_port       = 80;
  cfg.max_uri_handlers  = 8;
  cfg.recv_wait_timeout = 5;
  cfg.send_wait_timeout = 5;
  cfg.stack_size        = 8192;

  if (httpd_start(&httpServer, &cfg) != ESP_OK) {
    Serial.println("[ERRO] Servidor HTTP falhou");
    return;
  }

  const httpd_uri_t uris[] = {
    { "/",        HTTP_GET, indexHandler,   NULL },
    { "/stream",  HTTP_GET, streamHandler,  NULL },
    { "/capture", HTTP_GET, captureHandler, NULL },
    { "/flash",   HTTP_GET, flashHandler,   NULL },
    { "/res",     HTTP_GET, resHandler,     NULL },
  };
  for (const auto &u : uris) httpd_register_uri_handler(httpServer, &u);

  serverStarted = true;
  Serial.printf("[OK] Servidor HTTP na porta 80 — http://%s\n",
                WiFi.localIP().toString().c_str());
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-CAM Fast Stream ===");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!initCamera()) {
    // Sem delay(): agenda reinício via esp_timer one-shot de 5 s
    // Isso libera o setup() imediatamente e não congela nada.
    esp_timer_handle_t t;
    esp_timer_create_args_t ta = {};
    ta.callback = [](void*){ ESP.restart(); };
    ta.name     = "reboot";
    esp_timer_create(&ta, &t);
    esp_timer_start_once(t, 5000000ULL); // 5 s em µs
    Serial.println("[ERRO] Camera falhou — reiniciando em 5 s...");
    return; // setup() retorna; loop() roda normalmente enquanto aguarda
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Conexão inicial aguardada com millis() — sem delay()
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < 15000UL) {
    // Apenas aguarda; o scheduler do FreeRTOS continua rodando tasks internas
    // (TCP/IP, Wi-Fi driver) porque não há delay() bloqueando a task do Arduino.
    taskYIELD();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERRO] Wi-Fi falhou — tentará reconectar no loop()");
    wifiReconnectAt = millis() + 10000UL; // agenda próxima tentativa
    return;
  }

  Serial.printf("[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
  startServer();
  Serial.println("=== Pronto! ===\n");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
// Tudo aqui é non-blocking: usa millis() para intervalos,
// nunca delay(). Não interfere em câmera, Wi-Fi ou TCP.
void loop() {
  uint32_t now = millis();

  // ── Reconexão Wi-Fi sem delay() ──────────────────────────────────────────
  if (WiFi.status() != WL_CONNECTED) {
    if (now >= wifiReconnectAt) {
      Serial.println("[WiFi] Reconectando...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiReconnectAt = now + 15000UL; // tenta de novo em 15 s se falhar
    }
    return; // não adianta checar servidor sem Wi-Fi
  }

  // ── Garante servidor rodando depois de uma reconexão ─────────────────────
  if (!serverStarted) {
    Serial.printf("[WiFi] Reconectado! IP: %s\n", WiFi.localIP().toString().c_str());
    startServer();
  }

  // ── Log de heap a cada 30 s (millis, sem delay) ───────────────────────────
  if (now - loopLastLog >= 30000UL) {
    loopLastLog = now;
    Serial.printf("[Heap] %u bytes livres | IP: %s\n",
                  esp_get_free_heap_size(),
                  WiFi.localIP().toString().c_str());
  }

  // Loop retorna imediatamente — FreeRTOS escala as outras tasks
  // (stream HTTP, Wi-Fi driver, watchdog) sem nenhum bloqueio.
}
