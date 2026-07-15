# XIAO ESP32-S3 迷你即時取景相機

用 **Seeed XIAO ESP32-S3 Sense(內建 OV2640 鏡頭)** 擷取畫面,即時顯示在
**2.4" ILI9341 SPI TFT(240×320,無觸控)** 上,做成一台小相機。

## 一、元件清單(照片中的東西)

| 元件 | 說明 | 在本專案的角色 |
|------|------|----------------|
| XIAO ESP32-S3 Sense | 主控 + OV2640 相機(鏡頭已在擴充板上) | 抓影像、驅動顯示器 |
| 2.4" ILI9341 SPI TFT | 240×320,SPI 介面,**不支援觸控** | 顯示即時畫面 |
| 搖桿模組(KY-023) | 類比 XY + 按鈕 | **選配**:按鈕凍結畫面 |
| 麵包板 | 免焊麵包板 | 固定接線 |
| 杜邦線 ×2 包 | 公對公 | 接線 |

> 重點:相機是透過 Sense 擴充板「內部」連到 XIAO(底部 B2B 連接器),
> **不會佔用外露的 D0~D10 排針**,所以顯示器與搖桿都能用外露腳位。

## 二、實作流程總覽

1. **裝好環境**:Arduino IDE + ESP32 開發板套件 + Arduino_GFX 函式庫。
2. **接線**:先只接顯示器(5 條訊號 + 電源),搖桿最後再接。
3. **燒錄基本顯示測試**(可選),確認顯示器會亮、顏色正常。
4. **燒錄 `camera_display.ino`**:相機 → 顯示器即時取景。
5. **微調**:畫面上下顛倒 / 顏色偏色,用程式裡的翻轉、Be 位元序選項調。
6. **(選配)接搖桿**,做凍結畫面等互動。

## 三、接線方式

XIAO 外露腳位的 **D 編號 ↔ GPIO 編號** 對照(程式碼裡用的是 GPIO 編號):

| D 編號 | GPIO | 用途 |
|--------|------|------|
| D0 | 1 | TFT RES(RESET) |
| D1 | 2 | TFT CS |
| D3 | 4 | TFT DC |
| D8 | 7 | TFT SCL(時脈) |
| D9 | 8 | (空著,此顯示器無 MISO) |
| D10 | 9 | TFT SDA(資料) |
| D2 | 3 | 搖桿 VRx(選配) |
| D4 | 5 | 搖桿 VRy(選配) |
| D5 | 6 | 搖桿 SW 按鈕(選配) |

### 顯示器接線表(依實機絲印:8 pin)

實機顯示器絲印為 `GND VCC SCL SDA RES DC CS BLK`(共 8 pin,**無 MISO**)。
`SCL/SDA` 是 SPI 的時脈/資料,**不是 I2C**,只是這家廠商的命名。

| 顯示器絲印 | 別名 | 接到 XIAO | 說明 |
|-----------|------|-----------|------|
| GND | GND | GND | 共地 |
| VCC | VCC | **3V3** | 用 3.3V(邏輯同為 3.3V,最安全) |
| SCL | SCK / CLK | D8 (GPIO7) | SPI 時脈 |
| SDA | MOSI / SDI | D10 (GPIO9) | SPI 資料 |
| RES | RST / RESET | D0 (GPIO1) | 重置 |
| DC | DC / RS | D3 (GPIO4) | 資料/命令 |
| CS | CS | D1 (GPIO2) | 晶片選擇 |
| BLK | LED / BL | **3V3** | 背光電源 |

> 此板無 MISO 腳,程式中 `TFT_MISO` 設為 `-1`,原本的 D9 空著即可。

> ⚠️ **電源提醒**:此模組請用 **3.3V** 供電(VCC 與 BLK 都接 3V3)。
> XIAO 的邏輯是 3.3V,若把 VCC 接 5V 而未做準位轉換,命令可能異常。
> 用 USB 供電時 XIAO 的 3V3 足以帶動這片顯示器與背光。

> 🔎 **驅動晶片**:外盒標示 ILI9341,程式預設用 `Arduino_ILI9341`。
> 若螢幕空白/亂碼,可能其實是 ST7789,把程式裡的 `Arduino_ILI9341`
> 改成 `Arduino_ST7789` 再燒錄即可(這類 8-pin SCL/SDA 藍板常見兩種晶片)。

### 搖桿(KY-023,選配)接線表

| 搖桿腳位 | 接到 XIAO | 說明 |
|----------|-----------|------|
| GND | GND | 共地 |
| +5V | 3V3 | 供電(接 3.3V 即可,類比範圍 0~3.3V) |
| VRx | D2 (GPIO3) | X 軸類比(ADC) |
| VRy | D4 (GPIO5) | Y 軸類比(ADC) |
| SW | D5 (GPIO6) | 按鈕(內部上拉,按下為低電位) |

程式中 `USE_JOYSTICK` 設 `true` 才會啟用;沒接就設 `false`。

### 供電方式(推薦:行動電源 → XIAO → 分電)

最簡單的做法是用**行動電源經 USB-C 供電給 XIAO**,再由 XIAO 分電給其他元件:

```
行動電源 ──USB-C──► XIAO USB 孔
                     │
                     ├─ 3V3 腳 ──► 顯示器 VCC / LED、搖桿 +
                     └─ GND 腳 ──► 顯示器 GND、搖桿 GND(共地)
```

- **顯示器 VCC/LED 接 `3V3`**(不要接 5V,邏輯才與 XIAO 的 3.3V 一致)。
- **共地**:所有元件的 GND 都回到 XIAO 的 GND(全接在 XIAO 上本來就共地)。
- **電流預算**:相機 + 顯示器 + 背光約 250~350mA,XIAO 板上 3V3 穩壓器帶一片
  ILI9341 綽綽有餘。
- **行動電源自動斷電**:部分行動電源在耗流過低(<50~100mA)時會自動關機;
  本專案「相機 + 背光」耗流遠高於此門檻,正常運作不會被關掉。
- ⚠️ `3V3` 是**輸出**腳:只由 USB 端供電後往外送,**切勿**再從 `3V3` 灌入
  另一顆外部電源,以免電源打架。維持「單一來源(行動電源)→ XIAO → 分電」最安全。
- (進階)XIAO ESP32-S3 板子背面有 BAT+/BAT− 焊墊可直接接鋰電池,
  但用行動電源走 USB 是最省事、免焊的做法。

## 四、軟體環境設定

1. **安裝 ESP32 開發板套件**
   Arduino IDE → 偏好設定 → 額外開發板管理員網址加入:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   然後在「開發板管理員」安裝 **esp32 by Espressif**(建議 3.x)。

2. **安裝函式庫**(程式庫管理員搜尋安裝)
   - **GFX Library for Arduino**(作者 Moon On Our Nation)— 驅動顯示器
   - **TJpg_Decoder**(作者 Bodmer)— 相簿/錄影預覽解碼 JPEG
   > 選 Arduino_GFX 的原因:所有腳位/驅動都寫在 `.ino` 裡,不用改函式庫檔。

3. **開發板設定(工具選單)**
   - 開發板:`XIAO_ESP32S3`
   - **PSRAM:`OPI PSRAM`** ← 必開,否則相機 frame buffer 配不到記憶體
   - USB CDC On Boot:`Enabled`(方便看 Serial 訊息)

4. **燒錄**:把 `camera_display.ino` 與 `camera_pins.h` 放同一資料夾開啟,
   選好序列埠 → 上傳。

## 五、運作原理(簡述)

- 相機設定成 `PIXFORMAT_RGB565` + `FRAMESIZE_QVGA`(320×240),
  輸出的每個像素就是顯示器能直接吃的 16-bit 色。
- 顯示器 `rotation=1`(橫向)後解析度是 320×240,和相機輸出剛好一致,
  所以每張影像可以整塊 `draw16bitBeRGBBitmap()` 推上去,不必縮放。
- 相機 RGB565 是**大端序(byte-swapped)**,用 `...BeRGBBitmap()` 版本畫,
  顏色才不會偏。一張 QVGA 影像放在 PSRAM(約 150 KB),雙緩衝讓畫面較順,
  實測約 10~20 fps。

## 六、常見問題排解

| 症狀 | 可能原因 / 解法 |
|------|-----------------|
| 顯示器全白 / 沒反應 | 檢查 CS/DC/RES/SCL/SDA 是否接對;VCC、GND、BLK 是否有接 |
| 顯示器空白 / 亂碼(接線都對) | 晶片可能是 ST7789,把 `Arduino_ILI9341` 改成 `Arduino_ST7789` |
| 開機顯示 `Camera init FAIL` | PSRAM 沒設成 `OPI PSRAM`;或開發板沒選 `XIAO_ESP32S3` |
| **拍的照片損毀/雜訊**(即時預覽卻正常) | XIAO Sense 已知坑:XCLK 要用 **10MHz**,20MHz 會產生損毀影格(`EV-VSYNC-OVF`) |
| **相簿/影片預覽紅藍互換** | `TJpgDec.setSwapBytes(false)`(搭配 Arduino_GFX 的 `draw16bitRGBBitmap`) |
| 畫面上下/左右顛倒 | 改 `initCamera()` 裡的 `set_vflip` / `set_hmirror`(0/1) |
| 即時取景顏色不對(紅藍互換) | 把 `draw16bitBeRGBBitmap` 換成 `draw16bitRGBBitmap`(位元序切換) |
| 畫面太暗 / 背光不亮 | 確認 BLK 有接 3V3;供電請用 USB 而非弱電池 |
| 更新很卡 | 確認用硬體 SPI(本範例已是);可試著提高 SPI 時脈或維持 QVGA |

## 六之一、搖桿操作(功能表)

```
相機模式(即時取景)
  ├─ 點按搖桿鍵  → 拍照(存 JPEG 到 SD)
  └─ 長按搖桿鍵  → 開啟功能表
功能表(搖桿上下選、點按進入、長按回相機)
  ├─ Filter  濾鏡  → 左右切特效(Normal/Negative/B&W/Red/Green/Blue/Sepia)
  │                  上下調亮度、點按返回
  ├─ Gallery 相簿  → 左右翻看 SD 卡照片、點按返回
  ├─ Video   錄影  → 點按開始/停止錄影、長按返回
  └─ Back    回相機
```

- **錄影格式**:MJPEG 存成 `.avi`(`/VID00000.AVI`…)。**無聲音**、約 5–15fps、
  播放速度依實際幀率自動校正。用 VLC 等播放器可直接開。
- **方向相反**:若搖桿上下或左右操作相反,調 `pollJoyDir()` 裡的門檻對調即可。
- 搖桿接線:VRx→D2、VRy→D4、SW→D5、`+`→3V3、GND→GND。

## 六之二、加裝 microSD 卡拍照

- **格式**:格式化成 **FAT32**(32GB 建議 FAT32,**勿用 exFAT**,ESP32 常掛載失敗)。
  Mac:磁碟工具 → 清除 → 格式「MS-DOS (FAT)」、架構「主開機記錄(MBR)」。
- **接線**:XIAO ESP32-S3 Sense 內建 SD 卡槽,**不用外接線**,插卡即可。
  SD 卡與顯示器**共用 SPI**:SCK=D8、MOSI=D10、**MISO=D9**、SD_CS=GPIO21(內部)。
- **程式**:顯示器改用 `Arduino_HWSPI`(共享匯流排),`TFT_MISO` 設回 D9(GPIO8)。
- **操作**:即時取景時**按一下搖桿按鈕**,把當前畫面存成 `/IMG00000.JPG`(依序遞增),
  螢幕上方會顯示存檔結果。無卡或寫入失敗會顯示紅字提示。
- **拍照方式**:按下時暫時切到相機**原生 JPEG** 輸出再存(顏色正確、和錄影同一條路),
  存完切回 RGB565 取景。因此拍照會頓約 0.5–1 秒屬正常。

## 七、可延伸方向

- **拍照存檔**:XIAO Sense 有 microSD 卡槽,可在按下搖桿按鈕時把當前
  frame 存成 JPEG(改用 `PIXFORMAT_JPEG` 擷取後存檔,顯示時再切回 RGB565)。
- **搖桿選單**:用 VRx/VRy 讀類比值切換濾鏡、亮度、白平衡
  (`esp_camera_sensor_get()` 取得 `sensor_t` 後可調)。
- **Wi-Fi 串流**:同一顆 XIAO 也能同時跑 MJPEG 網頁串流,手機看即時畫面。

## 檔案

- `camera_display.ino` — 主程式
- `camera_pins.h` — OV2640 for XIAO ESP32-S3 Sense 腳位定義
