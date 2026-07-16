// camera_pins.h
// 相機腳位定義 for Seeed Studio XIAO ESP32-S3 Sense
// OV2640 與 OV5640 共用同一排線接頭,腳位相同(驅動自動偵測型號),本檔通用
// 這些腳位是「Sense」擴充板透過底部 B2B 連接器內部走線，
// 不會佔用外露的 D0~D10 排針，所以顯示器/搖桿可自由使用外露腳位。

#pragma once

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40   // I2C SDA (SCCB)
#define SIOC_GPIO_NUM   39   // I2C SCL (SCCB)

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15

#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13
