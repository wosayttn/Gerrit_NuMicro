# Nuvoton Audio GUI Firmware (NAU88C22 Platform)

> 整合 USB Audio + USB HID 遠端控制 + I2C Codec 設定 + I2S/PDMA 音訊傳輸的評估與調試韌體。

## 1. 專案定位

本韌體運行於 Nuvoton M55M1 MCU，透過 USB 連接 PC：

- 提供 **USB Audio Class**：可被系統辨識為標準播放/錄音裝置。
- 提供 **USB HID 通道**：搭配 *Nuvoton Audio GUI* 進行 NAU88C22（兼容 NAU88L25/L21/85L40 等）暫存器與 I2S 參數即時操作。
- 透過 **I2C** 初始化與控制 Codec，並以 **I2S + PDMA** 進行高效率音訊資料搬運。

適用於：Codec 評估、音訊路徑驗證、參數調整、自動化測試腳本。

## 2. 系統架構概覽

```text
PC (Nuvoton Audio GUI / 播放/錄音應用)
  │
  ├─ USB HID  → 控制/查詢指令 → MCU 解析 → I2C → Codec 寄存器
  │
  └─ USB Audio (Isochronous) ←→ Ring Buffer ←→ PDMA ←→ I2S ←→ Codec DAC/ADC ←→ 類比音訊
```

核心模組關係：

```text
main.c                : 入口 / 主循環 / HID 指令分派 / 啟停音訊
usbd_audio.c          : HID 指令分幀與解析 + 播放/錄音緩衝管理
codec_config.c/h      : Codec 初始化序列與暫存器寫入 API
i2c_process.c/h       : I2C 狀態機 (START/SLA/ACK/NACK/READ/WRITE/Timeout)
descriptors.c         : USB Device/Configuration/Interface/Endpoint 描述符
audio_class.h         : 音訊通道、採樣率、封包大小、緩衝與常數
user_config.h         : 硬體資源選擇 (I2C/I2S/PDMA IDs…)
sys_init.c            : 時鐘/模組啟動 (192 MHz 核心 / APLL 撥頻)
DemoSequence_*.txt    : 參考寄存器配置腳本
```

## 3. 啟動流程 (Boot Flow)

```text
上電/Reset
  → SYS_INIT()：時鐘/模組啟用 (HIRC/HXT/APLL0→192MHz)
  → UART / PDMA / Flash / Systick 初始化
  → 延遲 2 秒 (電源/類比穩定)
  → NAU88C22_CodecMst_Init(48000)：重置→慢充→靜音→路徑→時鐘→解除靜音
  → I2C_Init / I2S_Init / HSUSBD_Init
  → 進入 while(1) 主循環：PC_ProcHIDCmd() 處理 HID 指令
  → 播放/錄音啟動：AudioDeviceCtrl() 依旗標控制 I2S + PDMA
```

## 4. 主要功能詳解

### 4.1 Codec 初始化策略

- 分階段：軟體重置 → 低壓模式配置 → 慢充 (IO Buffer / Reference) → 先全部靜音 + ZC → 數位介面與取樣率 → 啟用路徑 → 延遲 → 同步解除靜音。
- 使用 `I2C_WriteNAU88C22(addr, data)`；批次配置可依照 `DemoSequence_NAU88C22_MasterMode.txt`。

### 4.2 HID 指令分類 (hid_transfer.h)

| 類型 | 用途 |
|------|------|
| HIDTRANS_CMD_NUVOTON | 結構化 Codec 操作者指令 (可含內部語意) |
| HIDTRANS_CMD_I2C | 直接 I2C 腳本模式 (寄存器腳本) |
| HIDTRANS_CMD_CODEC_I2S_SETTING | I2S 參數設定 (採樣率/通道/格式/位寬/FrameSize) |
| HIDTRANS_CMD_CODEC_BURST_DATA_* | Burst 多筆寄存器寫入 (8/9/16 bit) |
| HIDTRANS_CMD_FW_VERSION | 韌體版本查詢 (g_u32PID) |
| HIDTRANS_CMD_BOARD_STATE / BOARD_NUM | 板卡模式/編號 |
| HIDTRANS_CMD_DATA_CONFIG/PRESET | 韌體配置儲存/套用 |

分幀傳輸：每 64 bytes 一幀；`FrameNum bit7=1` 表示尚有後續資料。

### 4.3 I2C 通訊狀態機 (i2c_process.c)

- 處理 Status Code：0x08, 0x18, 0x28, 0x40, 0x50, 0x58 等。
- 支援寫後讀流程 / NACK 重試 / 超時旗標。
- 與 HID 緩衝協作：首 4 bytes 為 (Len, Type, RespondFlag...)。

### 4.4 音訊資料鏈

- Ring Buffer + Ping-Pong Buffer（播放與錄音各自）：
  - 播放：USB OUT → Play Ring → PDMA → I2S TX → Codec DAC
  - 錄音：Codec ADC → I2S RX → PDMA → Rec Ring → USB IN
- 水位監控：上/下閾值避免 Underflow / Overflow（自適應模式可切換）。

### 4.5 參數動態設定 (I2S)

HID 寫入後更新：

- `g_u32I2S0SampleRate` (8k/16k/32k/48k/96k)
- `g_u32I2S0Channel` (2CH / 4CH)
- `g_u32I2S0Format` (I2S / I2S_MSB / PCM / PCM_MSB)
- `g_u32I2S0WordSize` (16 / 24 / 32bit)
- `g_u32I2S0FrameSize` (32Fs / 64Fs)

並重新初始化緩衝：`PlaybackBuffInit()` / `RecordBuffInit()`。

## 5. 編譯與部署

### 5.1 開發環境

- IDE：Keil MDK (打開 `Keil/Codec_ControlBoard.uvprojx`)
- 燒錄：Nu-Link Debugger
- PC 工具：Nuvoton Audio GUI (Windows)

### 5.2 建置流程（Keil）

1. 開啟專案 `Codec_ControlBoard.uvprojx`
2. Build (F7) – 確認無錯誤
3. Download (F8) – 燒錄至開發板
4. 連接 USB 至 PC
5. 觀察 UART (115200-8-N-1) 顯示初始化訊息

### 5.3 GUI 使用流程

1. 啟動 Nuvoton Audio GUI → 自動發現 HID 裝置
2. 寄存器：單筆讀寫或載入腳本（`DemoSequence_*.txt`）
3. I2S 設定：選擇採樣率 / 通道 / 格式 → 套用
4. 測試播放：以系統播放音樂 → 耳機/喇叭輸出
5. 測試錄音：用系統錄音程序驗證輸入

## 6. 常見問題 (FAQ / Debug)

| 問題 | 可能原因 | 處理建議 |
|------|----------|----------|
| USB 未枚舉 | 供電或線材異常 / 描述符損壞 | 換線、重編譯、檢查 descriptors.c |
| 無音訊輸出 | Codec 初始化失敗 / I2S 參數不符 | 檢查 UART 訊息 / 重新套用 I2S 設定 |
| I2C 超時 | 線路 / 地址錯誤 / 上拉不足 | 量測 SCL/SDA、確認地址 0x1A |
| 播放卡頓 | Ring Buffer 下溢 | 增加 RING_BUFF_LEVEL 或改善 USB 傳輸節奏 |
| Burst 寫入失敗 | HID 分幀遺失 | 確認 FrameNum bit7 / PC 工具完整送出 |
| 讀不到資料 (Bongiovi 命令) | 未先送 Write | 需先寫入再讀回 |

## 7. 擴充建議

1. **多 Codec 支援**：抽象化初始化函式表。
2. **Profile 管理**：Flash 儲存多組音訊場景後在 GUI 切換。
3. **新增 DSP 範例**：EQ / RMS / AGC（放入 Ping-Pong 交換期間）。
4. **Loopback 測試模式**：加速產線驗證。
5. **更細錯誤碼**：將 I2C Timeout / NACK / HID Frame Error 分級回報。
6. **CRC 校驗**：對 Burst 大批資料加 CRC 提升可靠度。

## 8. 關鍵全域變數 (節選)

| 變數 | 功能 |
|------|------|
| `g_au32USBPlayRingBuff[]` | 播放端 USB→I2S 環形緩衝 |
| `g_au32USBRecRingBuff[]` | 錄音端 I2S→USB 環形緩衝 |
| `g_u32I2S0SampleRate` | 目前 I2S 取樣率 |
| `g_u32I2S0Channel` | I2S 通道模式 (2CH/4CH) |
| `g_u8HIDCommandProcessing` | HID 處理狀態機旗標 |
| `g_u8I2CTimeOut` | I2C 超時結果 |
| `g_u8WriteFail` | Codec 寫入結果回報 |

## 9. 自訂 HID 指令範例 (示意)

```c
// hid_transfer.h
#define HIDTRANS_CMD_CUSTOM 0x20

// usbd_audio.c -> HIDTrans_ProcessCommand()
else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CUSTOM) {
    // 解析 pu8Buffer[4..] 執行自訂操作
    g_u8HIDCommandProcessing = FINISHED;
}
```

## 10. 安全 / 音質保護策略

- 初始化全程使用靜音 + Zero-Cross 避免爆音。
- 延遲步驟保證類比穩定與偏壓建立。
- Gain 解除採用「同步解除 + 更新位」方式平滑恢復音量。

## 11. 授權與版權

- 原始碼授權：SPDX-License-Identifier: Apache-2.0
- 版權：© 2025 Nuvoton Technology Corp. All rights reserved。

## 12. 最精簡總結

本專案 = 一個可透過 GUI 即時操作 NAU88C22 的 USB 音訊 + HID 控制 + I2C/I2S/PDMA 整合式韌體平台，適合評估、驗證與量產前參數調校。

---
如需：英文版 README、加入 ASCII 流程圖圖片化、或新增快速腳本區段，歡迎再提出需求。
