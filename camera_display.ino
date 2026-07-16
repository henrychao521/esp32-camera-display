/*
 * camera_display.ino
 * ------------------------------------------------------------------
 * 迷你相機:XIAO ESP32-S3 Sense (OV2640 或 OV5640) → 2.4" ILI9341 SPI TFT
 *   換 OV5640:接線/腳位不變(自動偵測),只需 CAM_XCLK_HZ 設 20000000
 *   相機模式 :即時取景。點按搖桿=拍照;長按=開功能表
 *   功能表   :搖桿上下選、點按進入、長按回相機
 *     - Filter   濾鏡 :左右切特效、上下調亮度、點按返回
 *     - Gallery  相簿 :左右翻看 SD 照片、點按返回
 *     - Video    錄影 :點按開始/停止錄 MJPEG(.avi)、長按返回
 *     - WiFi Send 傳檔:開熱點,手機連上瀏覽/下載照片(Captive Portal),
 *                       連上時用手機時間對時 → 之後拍照寫入 EXIF 時間;含 OTA 更新
 *     - Back     回相機
 *
 * 函式庫:
 *   - GFX Library for Arduino (Arduino_GFX)
 *   - TJpg_Decoder (Bodmer)   ← 相簿/錄影預覽解碼 JPEG,需另裝
 *   - esp_camera / SD 內建於 ESP32 核心(3.x)
 * 開發板:XIAO_ESP32S3 / PSRAM: OPI PSRAM(必開) / USB CDC On Boot: Enabled
 * SD 卡 :FAT32(勿用 exFAT)
 * ------------------------------------------------------------------
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <Arduino_GFX_Library.h>
#include <TJpg_Decoder.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include <time.h>
#include <sys/time.h>
#include "camera_pins.h"

// ============ 顯示器 + SD 共用 SPI(GPIO 編號)============
#define TFT_CS    2    // D1  → 顯示器 CS
#define TFT_RST   1    // D0  → 顯示器 RES
#define TFT_DC    4    // D3  → 顯示器 DC
#define TFT_SCK   7    // D8  → 顯示器 SCL(與 SD 共用)
#define TFT_MOSI  9    // D10 → 顯示器 SDA(與 SD 共用)
#define TFT_MISO  8    // D9  → SD 卡 MISO(顯示器不接)
#define SD_CS     21   // 內建 SD 卡 CS(內部腳位)

// ============ 搖桿 ============
#define JOY_X     3    // D2 (GPIO3) → VRx(類比)
#define JOY_Y     5    // D4 (GPIO5) → VRy(類比)
#define JOY_SW    6    // D5 (GPIO6) → 按鈕(低電位有效)

// ============ 顏色(RGB565)============
#define COL_BLACK 0x0000
#define COL_WHITE 0xFFFF
#define COL_RED   0xF800
#define COL_GREEN 0x07E0
#define COL_BLUE  0x051D
#define COL_GREY  0x8410

// 顯示器用 Arduino_HWSPI(共享匯流排,才能和 SD 卡共用同一組 SPI)
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, TFT_RST, 1 /* 橫向 */, false);

// ============ 狀態 ============
enum Mode { LIVE, MENU, FILTER, GALLERY, VIDEO, WIFIXFER };
Mode mode = LIVE;

enum { DIR_NONE, DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };

bool cameraOK = false, sdOK = false;
int  photoIndex = 0;
int  menuSel = 0;
const char *menuItems[] = {"Filter", "Gallery", "Video", "WiFi Send", "Back to Camera"};
const int   MENU_N = 5;

bool rtcValid = false;               // RTC 是否已對時(手機瀏覽器連上時設定)

// ============ 相機 ============
// OV5640 建議 20MHz;若換回 OV2640 或看到損毀影格(EV-VSYNC-OVF),改成 10000000
#define CAM_XCLK_HZ 20000000
// 畫面方向(顛倒/鏡像就改這兩個):OV5640→VFLIP 0、OV2640→VFLIP 1
#define CAM_VFLIP   0     // 上下翻(0/1)
#define CAM_HMIRROR 0     // 左右鏡像(0/1)

// ============ 螢幕自動關閉 ============
#define SCREEN_TIMEOUT_MS 30000UL    // 無操作幾毫秒後關螢幕(30 秒;改這裡調整)
#define BLK_PIN 43                   // 顯示器 BLK 接 D6(GPIO43),可真正關/開背光(-1=只關顯示,BLK 接 3V3)
unsigned long lastActivityMs = 0;
bool screenOff = false;

// 濾鏡
int curEffect = 0, curBright = 0;
const char *effectNames[] = {"Normal", "Negative", "B&W", "Red", "Green", "Blue", "Sepia"};

// 相簿
#define GAL_MAX 120
char galList[GAL_MAX][16];
int  galCount = 0, galIdx = 0;

// 錄影(AVI/MJPEG)
File aviFile;
bool recording = false;
uint32_t aviFrames = 0, aviMovi = 0, aviStartMs = 0;
char aviPath[16];

// ------------------------------------------------------------------
// 相機
void applySensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_vflip(s, CAM_VFLIP);       // 上下翻(見頂部 CAM_VFLIP)
  s->set_hmirror(s, CAM_HMIRROR);   // 左右鏡像(見頂部 CAM_HMIRROR)
  s->set_special_effect(s, curEffect);
  s->set_brightness(s, curBright);
}

bool initCamera(pixformat_t fmt) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;   config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = CAM_XCLK_HZ;      // OV5640=20MHz / OV2640=10MHz(見頂部 CAM_XCLK_HZ)
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240(即時取景;相機會自動偵測 OV2640/OV5640)
  config.pixel_format = fmt;              // RGB565(取景) 或 JPEG(錄影)
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("相機初始化失敗:0x%x\n", err); return false; }
  applySensor();
  return true;
}

bool setCamFormat(pixformat_t fmt) {
  esp_camera_deinit();
  return initCamera(fmt);
}

// ------------------------------------------------------------------
// 小工具
void centerText(const char *msg, uint16_t color = COL_WHITE) {
  gfx->setTextSize(2);
  gfx->setTextColor(color);
  gfx->setCursor(20, gfx->height() / 2 - 8);
  gfx->print(msg);
}

void toast(const char *msg, uint16_t color) {
  gfx->fillRect(0, 0, gfx->width(), 26, COL_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(color);
  gfx->setCursor(6, 5);
  gfx->print(msg);
}

// TJpg 解碼輸出到螢幕
bool jpgDraw(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  gfx->draw16bitRGBBitmap(x, y, bitmap, w, h);
  return true;
}

// ------------------------------------------------------------------
// 輸入:按鈕(0=無 / 1=點按 / 2=長按)。狀態用全域,喚醒時可 resetInput() 清掉
bool btnPressed = false, btnLong = false;
unsigned long btnT0 = 0;
bool joyArmed = true;

int pollButton() {
  bool low = (digitalRead(JOY_SW) == LOW);
  unsigned long now = millis();
  if (low && !btnPressed) { btnPressed = true; btnLong = false; btnT0 = now; return 0; }
  if (low && btnPressed && !btnLong && (now - btnT0 >= 800)) { btnLong = true; return 2; }
  if (!low && btnPressed) { btnPressed = false; if (!btnLong && (now - btnT0 < 800)) return 1; }
  return 0;
}

// 輸入:搖桿方向(需回中才會再觸發一次)
int pollJoyDir() {
  int x = analogRead(JOY_X), y = analogRead(JOY_Y);
  int dir = DIR_NONE;
  if (y < 900) dir = DIR_UP;           // 若上下相反,對調 UP/DOWN 門檻
  else if (y > 3200) dir = DIR_DOWN;
  else if (x < 900) dir = DIR_LEFT;    // 若左右相反,對調 LEFT/RIGHT
  else if (x > 3200) dir = DIR_RIGHT;
  if (dir == DIR_NONE) { joyArmed = true; return DIR_NONE; }
  if (!joyArmed) return DIR_NONE;
  joyArmed = false;
  return dir;
}

// 清掉輸入狀態(喚醒後呼叫,避免喚醒的那一下被當成操作)
void resetInput() { btnPressed = false; btnLong = false; joyArmed = false; }

// ------------------------------------------------------------------
// 拍照(RGB565 → JPEG → SD)
int nextIndex(const char *fmt) {
  int i = 0; char path[16];
  while (i < 99999) { snprintf(path, sizeof(path), fmt, i); if (!SD.exists(path)) break; i++; }
  return i;
}

// 建立 EXIF(APP1)區段:寫入相機型號與拍攝時間(dt = "YYYY:MM:DD HH:MM:SS")
static uint8_t exifBuf[220];
size_t buildExif(const char *dt) {
  uint8_t t[160]; int p = 0;
#define W8(v)  (t[p++] = (uint8_t)(v))
#define W16(v) do{ uint16_t _v=(v); t[p++]=_v&0xFF; t[p++]=_v>>8; }while(0)
#define W32(v) do{ uint32_t _v=(v); t[p++]=_v&0xFF; t[p++]=(_v>>8)&0xFF; t[p++]=(_v>>16)&0xFF; t[p++]=(_v>>24)&0xFF; }while(0)
  W8('I'); W8('I'); W16(0x2A); W32(8);          // TIFF 標頭(小端序),IFD0 在 offset 8
  W16(4);                                       // IFD0:4 個項目
  W16(0x010F); W16(2); W32(6);  W32(62);        // Make    → offset 62
  W16(0x0110); W16(2); W32(19); W32(68);        // Model   → offset 68
  W16(0x0132); W16(2); W32(20); W32(87);        // DateTime→ offset 87
  W16(0x8769); W16(4); W32(1);  W32(107);       // Exif IFD 指標 → offset 107
  W32(0);                                       // 無下一個 IFD
  memcpy(t + p, "Seeed", 6);               p += 6;    // 62
  memcpy(t + p, "XIAO ESP32S3 Sense", 19); p += 19;   // 68
  memcpy(t + p, dt, 19); t[p + 19] = 0;    p += 20;   // 87  DateTime
  W16(1);                                       // Exif IFD:1 個項目
  W16(0x9003); W16(2); W32(20); W32(125);       // DateTimeOriginal → offset 125
  W32(0);
  memcpy(t + p, dt, 19); t[p + 19] = 0;    p += 20;   // 125 DateTimeOriginal
#undef W8
#undef W16
#undef W32
  size_t tiffLen = p;                           // = 145
  int q = 0;
  exifBuf[q++] = 0xFF; exifBuf[q++] = 0xE1;     // APP1 標記
  uint16_t app1 = 2 + 6 + tiffLen;
  exifBuf[q++] = app1 >> 8; exifBuf[q++] = app1 & 0xFF;   // 長度(大端序)
  memcpy(exifBuf + q, "Exif\0\0", 6); q += 6;
  memcpy(exifBuf + q, t, tiffLen);    q += tiffLen;
  return q;
}

// 拍照:暫時切到相機「原生 JPEG」輸出再存檔(顏色正確,和錄影同一條路,
// 不經 RGB565→JPEG 轉換,徹底避開位元組順序問題)
void savePhoto() {
  if (!sdOK) { toast("No SD", COL_RED); delay(600); return; }
  toast("Saving...", COL_WHITE);
  if (!setCamFormat(PIXFORMAT_JPEG)) {
    toast("Cam switch fail", COL_RED); delay(600);
    setCamFormat(PIXFORMAT_RGB565); return;
  }
  camera_fb_t *fb = NULL;
  for (int i = 0; i < 4; i++) {           // 丟掉前幾張,讓曝光/白平衡穩定
    if (fb) esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
    delay(40);
  }
  bool ok = false;
  if (fb) {
    char path[16]; snprintf(path, sizeof(path), "/IMG%05d.JPG", photoIndex);
    File f = SD.open(path, FILE_WRITE);
    if (f) {
      if (rtcValid && fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
        time_t now = time(NULL); struct tm ti; localtime_r(&now, &ti);
        char dt[24]; strftime(dt, sizeof(dt), "%Y:%m:%d %H:%M:%S", &ti);
        size_t el = buildExif(dt);
        f.write(fb->buf, 2);                 // SOI
        f.write(exifBuf, el);                // APP1 EXIF(時間)
        f.write(fb->buf + 2, fb->len - 2);   // 其餘 JPEG
      } else {
        f.write(fb->buf, fb->len);           // 未對時 → 原樣存
      }
      f.close(); ok = true; photoIndex++;
    }
    esp_camera_fb_return(fb);
  }
  setCamFormat(PIXFORMAT_RGB565);         // 切回即時取景
  toast(ok ? "Saved" : "Save fail", ok ? COL_GREEN : COL_RED);
  delay(600);
}

// ------------------------------------------------------------------
// 功能表
void drawMenu() {
  gfx->fillScreen(COL_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(10, 8); gfx->print("MENU");
  gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
  gfx->setCursor(120, 14); gfx->print("up/down + press");
  for (int i = 0; i < MENU_N; i++) {
    int y = 44 + i * 42;
    if (i == menuSel) { gfx->fillRect(6, y - 5, gfx->width() - 12, 34, COL_BLUE); gfx->setTextColor(COL_WHITE); }
    else gfx->setTextColor(COL_GREY);
    gfx->setTextSize(2); gfx->setCursor(16, y); gfx->print(menuItems[i]);
  }
}

// ------------------------------------------------------------------
// 相簿
void buildGalleryList() {
  galCount = 0;
  File root = SD.open("/");
  if (!root) return;
  File f = root.openNextFile();
  while (f && galCount < GAL_MAX) {
    if (!f.isDirectory()) {
      const char *n = f.name();
      int L = strlen(n);
      if (L >= 4 && (strcasecmp(n + L - 4, ".JPG") == 0)) {
        if (n[0] != '/') { galList[galCount][0] = '/'; strncpy(galList[galCount] + 1, n, 14); }
        else strncpy(galList[galCount], n, 15);
        galList[galCount][15] = 0;
        galCount++;
      }
    }
    f = root.openNextFile();
  }
  root.close();
}

void showPhoto(int idx) {
  gfx->fillScreen(COL_BLACK);
  if (galCount == 0) { centerText("No photos", COL_GREY); return; }
  File f = SD.open(galList[idx]);
  if (!f) { centerText("Open fail", COL_RED); return; }
  size_t sz = f.size();
  uint8_t *buf = (uint8_t *)ps_malloc(sz);
  if (!buf) buf = (uint8_t *)malloc(sz);
  if (!buf) { f.close(); centerText("Mem fail", COL_RED); return; }
  size_t got = 0;                          // 讀滿整個檔(共享 SPI 下單次 read 可能不完整)
  while (got < sz) { int r = f.read(buf + got, sz - got); if (r <= 0) break; got += r; }
  f.close();
  TJpgDec.drawJpg(0, 0, buf, got);
  free(buf);
  gfx->fillRect(0, 0, 96, 20, COL_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(4, 3); gfx->printf("%d/%d", idx + 1, galCount);
}

// ------------------------------------------------------------------
// AVI / MJPEG 寫檔
void wr32(File &f, uint32_t v) { f.write((uint8_t)v); f.write((uint8_t)(v >> 8)); f.write((uint8_t)(v >> 16)); f.write((uint8_t)(v >> 24)); }
void wr16(File &f, uint16_t v) { f.write((uint8_t)v); f.write((uint8_t)(v >> 8)); }
void seekWr32(File &f, uint32_t pos, uint32_t v) { f.seek(pos); wr32(f, v); }

void writeAviHeader(File &f) {
  f.write((const uint8_t *)"RIFF", 4); wr32(f, 0); f.write((const uint8_t *)"AVI ", 4);
  f.write((const uint8_t *)"LIST", 4); wr32(f, 192); f.write((const uint8_t *)"hdrl", 4);
  f.write((const uint8_t *)"avih", 4); wr32(f, 56);
  wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // usecPerFrame(@32) / maxBps / pad / flags
  wr32(f, 0); wr32(f, 0); wr32(f, 1); wr32(f, 0);   // totalFrames(@48) / initial / streams / bufsize
  wr32(f, 320); wr32(f, 240);                       // width / height
  wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // reserved[4]
  f.write((const uint8_t *)"LIST", 4); wr32(f, 116); f.write((const uint8_t *)"strl", 4);
  f.write((const uint8_t *)"strh", 4); wr32(f, 56);
  f.write((const uint8_t *)"vids", 4); f.write((const uint8_t *)"MJPG", 4);
  wr32(f, 0); wr16(f, 0); wr16(f, 0); wr32(f, 0);   // flags / prio / lang / initialFrames
  wr32(f, 1); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // scale(@128) / rate(@132) / start / length(@140)
  wr32(f, 0); wr32(f, 0); wr32(f, 0);               // bufsize / quality / sampleSize
  wr16(f, 0); wr16(f, 0); wr16(f, 320); wr16(f, 240); // rcFrame
  f.write((const uint8_t *)"strf", 4); wr32(f, 40);
  wr32(f, 40); wr32(f, 320); wr32(f, 240);
  wr16(f, 1); wr16(f, 24); f.write((const uint8_t *)"MJPG", 4);
  wr32(f, 320 * 240 * 3); wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);
  f.write((const uint8_t *)"LIST", 4); wr32(f, 0); f.write((const uint8_t *)"movi", 4); // moviSize(@216)
}

bool aviStart() {
  snprintf(aviPath, sizeof(aviPath), "/VID%05d.AVI", nextIndex("/VID%05d.AVI"));
  aviFile = SD.open(aviPath, FILE_WRITE);
  if (!aviFile) return false;
  writeAviHeader(aviFile);
  aviFrames = 0; aviMovi = 0; aviStartMs = millis();
  return true;
}

void aviAddFrame(uint8_t *buf, size_t len) {
  aviFile.write((const uint8_t *)"00dc", 4); wr32(aviFile, len);
  aviFile.write(buf, len);
  uint32_t w = 8 + len;
  if (len & 1) { aviFile.write((uint8_t)0); w++; }   // 補到偶數
  aviMovi += w; aviFrames++;
}

void aviEnd() {
  uint32_t el = millis() - aviStartMs; if (el < 1) el = 1;
  float fps = aviFrames * 1000.0f / el; if (fps < 1) fps = 1;
  uint32_t fileSize = 224 + aviMovi;
  seekWr32(aviFile, 4,   fileSize - 8);                    // RIFF size
  seekWr32(aviFile, 32,  (uint32_t)(1000000.0f / fps));    // usecPerFrame
  seekWr32(aviFile, 48,  aviFrames);                       // totalFrames
  seekWr32(aviFile, 132, (uint32_t)(fps + 0.5f));          // rate (scale=1)
  seekWr32(aviFile, 140, aviFrames);                       // stream length
  seekWr32(aviFile, 216, fileSize - 220);                  // movi size
  aviFile.close();
  Serial.printf("錄影完成 %s  %u 幀  %.1f fps\n", aviPath, aviFrames, fps);
}

// ==================================================================
// WiFi 傳輸(AP 熱點 + Captive Portal + 網頁相簿下載 + OTA)
// ==================================================================
WebServer server(80);
DNSServer dnsServer;
const char *AP_SSID = "XIAO-CAM";
const char *AP_PASS = "12345678";        // WPA2 需 ≥8 碼;可自行修改
uint32_t wifiFilesServed = 0;

String argF() {
  String f = server.hasArg("f") ? server.arg("f") : "";
  if (f.length() && f[0] != '/') f = "/" + f;
  return f;
}

void serveFile(bool download) {
  String f = argF();
  if (!f.length() || !SD.exists(f)) { server.send(404, "text/plain", "not found"); return; }
  File file = SD.open(f);
  if (!file) { server.send(500, "text/plain", "open fail"); return; }
  size_t sz = file.size();
  if (download) server.sendHeader("Content-Disposition", "attachment; filename=" + f.substring(1));
  server.streamFile(file, "image/jpeg");
  file.close();
  wifiFilesServed++; (void)sz;
}

void handleRoot() {
  buildGalleryList();
  String h = "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>";
  h += "<style>body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px}";
  h += "h1{font-size:18px}a{color:#6cf}.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:8px;margin-top:10px}";
  h += "a.c{background:#1c1c1c;border-radius:8px;overflow:hidden;text-decoration:none;color:#ccc}";
  h += "img{width:100%;display:block}.n{font-size:11px;padding:4px 6px}</style>";
  h += "<h1>XIAO Cam &middot; Photos (" + String(galCount) + ")</h1>";
  h += "<p><a href='/ota'>Firmware update (OTA)</a></p><div class=g>";
  for (int i = 0; i < galCount; i++) {
    String n = galList[i];
    h += "<a class=c href='/dl?f=" + n + "'><img loading=lazy src='/img?f=" + n + "'><div class=n>" + n.substring(1) + "</div></a>";
  }
  h += "</div><script>fetch('/settime?e='+Math.floor((Date.now()-new Date().getTimezoneOffset()*60000)/1000));</script>";
  server.send(200, "text/html", h);
}

void handleSetTime() {
  if (server.hasArg("e")) {
    time_t epoch = (time_t)strtoul(server.arg("e").c_str(), NULL, 10);
    struct timeval tv; tv.tv_sec = epoch; tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    rtcValid = true;
  }
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {   // Captive Portal:未知網址一律導回首頁,觸發手機自動彈窗
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void handleOtaForm() {
  server.send(200, "text/html",
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;background:#111;color:#eee;padding:16px'>"
    "<h2>OTA Update</h2><form method=POST action=/ota enctype=multipart/form-data>"
    "<input type=file name=f accept=.bin> <input type=submit value=Upload></form>"
    "<p><a style='color:#6cf' href='/'>&larr; back</a></p>");
}
void handleOtaDone() {
  bool ok = !Update.hasError();
  server.send(200, "text/html", ok ? "OK, rebooting..." : "Update FAILED");
  delay(800);
  if (ok) ESP.restart();
}
void handleOtaUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("OTA 開始:%s\n", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);   // 失敗多半是分割區沒 OTA
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("OTA 完成:%u bytes\n", up.totalSize);
    else Update.printError(Serial);
  }
}

void startWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  dnsServer.start(53, "*", WiFi.softAPIP());       // 所有 DNS → 本機(Captive Portal)
  server.on("/", handleRoot);
  server.on("/img", []() { serveFile(false); });
  server.on("/dl",  []() { serveFile(true); });
  server.on("/settime", handleSetTime);
  server.on("/ota", HTTP_GET, handleOtaForm);
  server.on("/ota", HTTP_POST, handleOtaDone, handleOtaUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  wifiFilesServed = 0;
}

void stopWiFi() {
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

void drawWifiIcon(int cx, int cy, uint16_t c) {     // 簡易 WiFi 圖示
  for (int r = 4; r <= 12; r += 4) gfx->drawCircle(cx, cy + 6, r, c);
  gfx->fillCircle(cx, cy + 6, 2, c);
  gfx->fillRect(cx - 14, cy + 8, 28, 10, COL_BLACK);  // 只留上半弧
  gfx->fillCircle(cx, cy + 6, 2, c);
}

void drawClockIcon(int cx, int cy, uint16_t c) {
  gfx->drawCircle(cx, cy, 8, c);
  gfx->drawLine(cx, cy, cx, cy - 5, c);
  gfx->drawLine(cx, cy, cx + 4, cy, c);
}

void drawWifiStatic() {
  gfx->fillScreen(COL_BLACK);
  drawWifiIcon(24, 10, COL_GREEN);
  gfx->setTextSize(2); gfx->setTextColor(COL_GREEN);
  gfx->setCursor(46, 10); gfx->print("WiFi Transfer");
  gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
  gfx->setCursor(10, 42); gfx->print("Join this hotspot, page opens automatically:");
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(10, 60);  gfx->printf("SSID: %s", AP_SSID);
  gfx->setCursor(10, 84);  gfx->printf("PASS: %s", AP_PASS);
  gfx->setTextColor(COL_BLUE);
  gfx->setCursor(10, 108); gfx->printf("http://%s", WiFi.softAPIP().toString().c_str());
  gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
  gfx->setCursor(10, gfx->height() - 12); gfx->print("press joystick = stop & back");
}

void drawWifiDynamic() {
  int y = 138;
  gfx->fillRect(0, y, gfx->width(), 44, COL_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(10, y); gfx->printf("Clients:%d  Files:%lu",
                                     WiFi.softAPgetStationNum(), (unsigned long)wifiFilesServed);
  drawClockIcon(20, y + 30, rtcValid ? COL_GREEN : COL_GREY);
  gfx->setCursor(36, y + 24);
  if (rtcValid) {
    time_t n = time(NULL); struct tm t; localtime_r(&n, &t);
    char b[24]; strftime(b, sizeof(b), "%m/%d %H:%M:%S", &t);
    gfx->setTextColor(COL_GREEN); gfx->print(b);
  } else { gfx->setTextColor(COL_GREY); gfx->print("time: open page"); }
}

// ------------------------------------------------------------------
void enterSelected() {
  switch (menuSel) {
    case 0: mode = FILTER; break;                // 相機已是 RGB565
    case 1: buildGalleryList(); galIdx = 0; mode = GALLERY; showPhoto(0); break;
    case 2:
      if (setCamFormat(PIXFORMAT_JPEG)) { recording = false; mode = VIDEO; gfx->fillScreen(COL_BLACK); }
      else { drawMenu(); toast("Cam switch fail", COL_RED); delay(800); }
      break;
    case 3: startWiFi(); mode = WIFIXFER; drawWifiStatic(); drawWifiDynamic(); break;  // WiFi Send
    case 4: mode = LIVE; break;                                                    // Back
  }
}

void livePreview() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { delay(20); return; }
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)fb->buf, fb->width, fb->height);
  esp_camera_fb_return(fb);
}

// ------------------------------------------------------------------
// 螢幕自動關閉 / 喚醒
void backlight(bool on) {
#if BLK_PIN >= 0
  digitalWrite(BLK_PIN, on ? HIGH : LOW);
#endif
}

void redrawCurrent() {               // 喚醒後依目前模式重畫
  switch (mode) {
    case MENU:     drawMenu(); break;
    case GALLERY:  showPhoto(galIdx); break;
    case WIFIXFER: drawWifiStatic(); drawWifiDynamic(); break;
    default:       gfx->fillScreen(COL_BLACK); break;  // LIVE/FILTER/VIDEO 下一輪自動重畫
  }
}

void screenSleep() {
  gfx->displayOff();                 // 關閉顯示輸出
  backlight(false);                  // 若 BLK 接 GPIO 則真正關背光
  screenOff = true;
}

void screenWake() {
  backlight(true);
  gfx->displayOn();
  while (digitalRead(JOY_SW) == LOW) delay(10);   // 等按鈕放開
  resetInput();                      // 清掉輸入,避免喚醒那下被當操作
  lastActivityMs = millis();
  screenOff = false;
  redrawCurrent();
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);
#if BLK_PIN >= 0
  pinMode(BLK_PIN, OUTPUT); digitalWrite(BLK_PIN, HIGH);   // 背光開
#endif
  lastActivityMs = millis();

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);      // 共享匯流排

  if (!gfx->begin()) Serial.println("顯示器初始化失敗");
  gfx->fillScreen(COL_BLACK);
  gfx->setTextColor(COL_WHITE); gfx->setTextSize(2);
  gfx->setCursor(10, 10); gfx->println("Booting...");

  // TJpg 解碼器
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);     // Arduino_GFX + draw16bitRGBBitmap 要用 false(true 會紅藍互換)
  TJpgDec.setCallback(jpgDraw);

  sdOK = SD.begin(SD_CS);
  if (sdOK) { photoIndex = nextIndex("/IMG%05d.JPG"); Serial.printf("SD 就緒,下一張 %d\n", photoIndex); }
  else Serial.println("SD 掛載失敗:檢查卡片/格式 FAT32");

  cameraOK = initCamera(PIXFORMAT_RGB565);
  if (!cameraOK) {
    gfx->fillScreen(COL_RED);
    gfx->setCursor(10, 10); gfx->println("Camera FAIL");
    gfx->setTextSize(1); gfx->setCursor(10, 40);
    gfx->println("PSRAM=OPI & board=XIAO_ESP32S3");
  }
}

// ------------------------------------------------------------------
void loop() {
  if (!cameraOK) { delay(500); return; }
  int btn = pollButton();
  int dir = pollJoyDir();
  bool anyInput = (btn || dir != DIR_NONE || digitalRead(JOY_SW) == LOW);

  // ---- 螢幕自動關閉 / 喚醒 ----
  if (screenOff) {
    if (anyInput) screenWake();      // 第一次操作只喚醒,不觸發動作
    else delay(30);
    return;
  }
  if (anyInput) lastActivityMs = millis();
  // 閒置逾時關螢幕;但「錄影中」與「WiFi 傳輸中」不關(以免中斷)
  bool noSleep = (mode == WIFIXFER) || (mode == VIDEO && recording);
  if (!noSleep && (millis() - lastActivityMs > SCREEN_TIMEOUT_MS)) {
    screenSleep();
    return;
  }

  switch (mode) {
    case LIVE:
      livePreview();
      if (btn == 1) { if (sdOK) savePhoto(); else { toast("No SD", COL_RED); delay(600); } }
      else if (btn == 2) { menuSel = 0; mode = MENU; drawMenu(); }
      break;

    case MENU:
      if (dir == DIR_UP)   { menuSel = (menuSel + MENU_N - 1) % MENU_N; drawMenu(); }
      if (dir == DIR_DOWN) { menuSel = (menuSel + 1) % MENU_N; drawMenu(); }
      if (btn == 1) enterSelected();
      else if (btn == 2) mode = LIVE;
      break;

    case FILTER:
      livePreview();
      if (dir == DIR_LEFT)  { curEffect = (curEffect + 6) % 7; applySensor(); }
      if (dir == DIR_RIGHT) { curEffect = (curEffect + 1) % 7; applySensor(); }
      if (dir == DIR_UP)    { if (curBright < 2)  { curBright++; applySensor(); } }
      if (dir == DIR_DOWN)  { if (curBright > -2) { curBright--; applySensor(); } }
      {
        int h = 28, y = gfx->height() - h;
        gfx->fillRect(0, y, gfx->width(), h, COL_BLACK);
        gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
        gfx->setCursor(4, y + 6); gfx->printf("%s  Bright:%d", effectNames[curEffect], curBright);
      }
      if (btn == 1) { mode = MENU; drawMenu(); }
      break;

    case GALLERY:
      if (galCount > 0) {
        if (dir == DIR_LEFT)  { galIdx = (galIdx + galCount - 1) % galCount; showPhoto(galIdx); }
        if (dir == DIR_RIGHT) { galIdx = (galIdx + 1) % galCount; showPhoto(galIdx); }
      }
      if (btn == 1) { mode = MENU; drawMenu(); }
      break;

    case VIDEO: {
      if (btn == 2) {                        // 長按=返回
        if (recording) { aviEnd(); recording = false; }
        setCamFormat(PIXFORMAT_RGB565);
        mode = MENU; drawMenu(); break;
      }
      if (btn == 1) {                        // 點按=開始/停止
        if (!recording) { if (sdOK && aviStart()) { recording = true; gfx->fillScreen(COL_BLACK); } }
        else { aviEnd(); recording = false; gfx->fillScreen(COL_BLACK); }
      }
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) { delay(10); break; }
      if (recording) {
        aviAddFrame(fb->buf, fb->len);
        gfx->fillRect(0, 0, gfx->width(), 30, COL_BLACK);
        gfx->fillCircle(14, 15, 7, COL_RED);
        gfx->setTextSize(2); gfx->setTextColor(COL_WHITE); gfx->setCursor(30, 8);
        gfx->printf("REC %u  %us", aviFrames, (millis() - aviStartMs) / 1000);
      } else {
        TJpgDec.drawJpg(0, 0, fb->buf, fb->len);   // 預覽
        gfx->fillRect(0, 0, gfx->width(), 20, COL_BLACK);
        gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
        gfx->setCursor(4, 6); gfx->print("VIDEO  press=REC  hold=back");
      }
      esp_camera_fb_return(fb);
      break;
    }

    case WIFIXFER: {
      dnsServer.processNextRequest();
      server.handleClient();
      static uint32_t lastDraw = 0;
      if (millis() - lastDraw > 800) { drawWifiDynamic(); lastDraw = millis(); }
      if (btn == 1 || btn == 2) { stopWiFi(); mode = MENU; drawMenu(); }
      break;
    }
  }
}
