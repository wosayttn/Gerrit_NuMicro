一、專案背景與目的
本專案為 Nuvoton M55M1 微控制器上的 NAU88C22（兼容 NAU88L25/NAU88L21/NAU85L40 等系列）音訊編解碼器「設定與測試韌體」。
透過 USB 連接 PC，提供：

USB Audio Class（播放/錄音）功能，方便直接用系統音訊驗證訊號鏈。
USB HID 介面與 Nuvoton Audio GUI 工具互動，進行暫存器讀寫、I2S 參數設定、批次腳本執行。
I2C 與 Codec 寄存器通信、初始化序列、音訊路徑控制。
I2S + PDMA 高效率音訊資料搬運（低 CPU 佔用）。
定位：一個「可視化 + 即時控制 + 穩定音訊傳輸」的 Codec 評估與開發平台。

二、整體系統架構概觀
資料與控制流：
PC(NuvotonAudioGUI)
→ USB HID (命令/設定)
→ MCU main.c: PC_ProcHIDCmd() 解析
→ usbd_audio.c: HIDTrans_ProcessCommand() 整理解碼
→ I2C 狀態機 i2c_process.c: I2C_Callback() 寫讀 Codec
→ codec_config.c: I2C_WriteNAU88C22()/NAU88C22_CodecMst_Init() 操作暫存器

音訊資料流：
PC 音訊播放 → USB Audio Isochronous OUT → Ring Buffer → PDMA → I2S TX → Codec DAC → 扬聲器/耳機
麥克風 → Codec ADC → I2S RX → PDMA → Ring Buffer → USB Audio Isochronous IN → PC 錄音

核心模組功能對照：

main.c: 系統進入點、初始化、HID 命令分派、I2S/PDMA 啟停邏輯。
usbd_audio.c: HID 命令接收、資料分幀、多類型指令解析、播放/錄音緩衝管理。
codec_config.c/h: Codec 初始化序列、單筆寄存器寫入 API、預設設定表。
i2c_process.c/h: I2C 狀態機（START/SLA/ACK/NACK/READ/WRITE/Timeout）。
descriptors.c: USB 裝置/組態/介面/端點/Audio/HID 描述符。
audio_class.h: 音訊參數、端點封包大小、採樣率/通道/緩衝計算常數。
user_config.h: 硬體資源對應（I2C/I2S/PDMA 模組、Flash 起始位址等）。
sys_init.c: 時鐘與模組啟用，設定 192MHz 系統頻率以匹配 48k/96k 音訊時脈倍數。
DemoSequence_NAU88C22_MasterMode.txt: 參考寄存器初始化腳本。

三、啟動與執行流程（文字流程圖）
上電 / 重置 → main.c: main()
系統時鐘與模組初始化 → SYS_INIT() → 開啟 HIRC/HXT/APLL0、GPIO/I2C/I2S/PDMA/USB。
UART 初始化（除錯輸出）。
等待 2 秒（讓外部電容與 Codec 電源平穩）。
執行 Codec 初始化：NAU88C22_CodecMst_Init(48000) 完成重置、供電、介面格式、增益靜音、安全延遲、解除軟靜音。
初始化 I2C/I2S/USB（含描述符設定與端點配置）。
進入主迴圈 while(1)：不斷呼叫 PC_ProcHIDCmd() 處理 GUI 下發的 HID 指令。
HID 指令分類：
寄存器讀寫/批次寫入 → 封裝 I2C 資料 → 啟動 I2C 狀態機 → 回傳結果。
I2S 參數設定 → 更新全域參數、重設緩衝、可能寫入 Flash。
版本、板卡號、狀態查詢 → 直接回覆。
播放/錄音啟動條件：USB Audio 端點資料量與旗標（g_u8USBPlayEn / g_u8USBRecEn）→ 透過 AudioDeviceCtrl() 啟動或停止 PDMA/I2S 傳輸。
系統運行中監控：
Ring Buffer 水位（避免 Underflow/Overflow）
HID 分幀（每 64 bytes，FrameNum bit7 判斷是否有後續）
I2C 超時/NACK 重試
I2S 參數是否改變 → 重新配置

四、功能詳解

1. Codec 初始化（codec_config.c）
依序做：軟體重置 → 低壓模式設定 → IO Buffer 啟動與慢充 → Analog Output 預設 Mute + ZC → 介面格式/I2S 設定 → DAC/ADC 相關路徑 → 延遲保護 → 解除靜音。
使用 I2C_WriteNAU88C22() 封裝 I2C 寫入序列。
依採樣率選擇時鐘分頻組合（48k, 44.1k, 16k 等）。
2. HID 命令（usbd_audio.c → HIDTrans_ProcessCommand()）
支援類型（節錄）：
HIDTRANS_CMD_NUVOTON：專屬結構化寄存器操作指令。
HIDTRANS_CMD_I2C：直接 I2C 腳本模式。
HIDTRANS_CMD_CODEC_I2S_SETTING：I2S 參數（採樣率/通道/格式/位寬/FrameSize）。
HIDTRANS_CMD_FW_VERSION：韌體版本讀取（g_u32PID）。
HIDTRANS_CMD_BOARD_STATE / BOARD_NUM：板卡模式 / 編號。
HIDTRANS_CMD_CODEC_BURST_DATA_*：批次大量寄存器寫入。
分幀處理：u8FrameNum & 0x80 → 代表還有後續資料。
3. I2C 狀態機（i2c_process.c）
判斷 Status Code：0x08, 0x18, 0x28, 0x40, 0x50, 0x58 等。
控制發送/接收流程、切換讀寫模式（寫後讀）。
超時/NACK 計數：重試次數達門檻則標記失敗。
與 HID 緩衝協作：資料框前綴 4 bytes (長度, 命令類型, 回應標誌)。
4. USB Audio 與緩衝（usbd_audio.c）
播放/錄音 Ring Buffer 與 Ping-Pong Buffer 初始化：PlaybackBuffInit() / RecordBuffInit()。
動態計算閾值（Upper/Lower threshold）以支援自適應控制。
靜態條件：
PLAY_RATE_48K / REC_RATE_48K
PLAY_CHANNELS 與 REC_CHANNELS（可 2 或 4）
位寬目前注記僅支援 16-bit (g_u8USBTxDataLen = USB_16bit)
音量/Mute 變數：g_usbd_PlayVolumeL/R、g_usbd_PlayMute 等。
5. I2S/PDMA 資料搬運
PDMA IRQ (PDMA0_IRQHandler) 中根據通道完成旗標更新讀寫指標。
使用 ping-pong 結構避免切換延遲。
音訊資料格式由 g_u32I2S0WordSize / g_u32I2S0Channel / FrameSize 決定。
五、操作使用指南（流程性建議）
環境準備
IDE：Keil MDK (打開 Codec_ControlBoard.uvprojx)
硬體：M55M1 + NAU88C22 評估板 + Nu-Link 調試器
PC 軟體：安裝 Nuvoton Audio GUI
編譯與燒錄
上電後觀察
UART 輸出（115200-8-N-1）顯示：
CPU 時脈
“Configure NAU88C22 ... [OK]”
韌體標題
GUI 操作
連接 USB → 裝置顯示為音訊硬體 + HID
在 GUI 中：
寄存器頁：讀取/寫入單筆或載入批次腳本（如 DemoSequence_NAU88C22_MasterMode.txt）
I2S 設定頁：調整採樣率/通道/格式/位寬 → 送出指令 → HID 回覆 ACK
Firmware Version/Board Info：快速查詢系統狀態
播放測試：PC 播放音樂 → 耳機/喇叭輸出
錄音測試：啟用錄音 → 取樣資料返送 PC
修改初始化流程（例）
在 codec_config.c 中調整：

新增自訂 HID 命令（示例構想）
在 hid_transfer.h 加入：
在 HIDTrans_ProcessCommand() 增加分支：
六、故障排查
現象 可能原因 處理建議
USB 未枚舉 線材或供電不足 換線/測量 5V/檢查晶振
播放無聲 Codec 未正確初始化 / I2S 參數不符 確認 NAU88C22_CodecMst_Init() 是否執行成功；檢查 GUI I2S 設定是否與 PC 播放格式一致
I2C 失敗 地址錯誤 / 佈線拉拒 / NACK 重試 量測 SCL/SDA 波形；查看 g_u8NackCnt 是否大量累積
音訊破音 Buffer 下溢或上溢 檢查 g_bUnderFlow / g_bOverFlow；調高 RING_BUFF_LEVEL 或檢查傳輸延遲
GUI 無法讀寄存器 HID 分幀未完成 / 先 Read 沒有先 Write 確認是否需先執行寫入配置；檢查 FrameNum bit7
改變採樣率後失效 未重新初始化 I2S 緩衝 呼叫 PlaybackBuffInit() / RecordBuffInit() 並確認 Switch_I2S_Sample_Rate() 被執行
七、安全與一致性設計要點
初始化階段先全靜音（避免爆音）。
延遲使用 CLK_SysTickDelay() 確保類比電路穩定。
HID 分幀保證大型批次資料完整性。
I2C exception（NACK/Timeout）具重試與狀態回報。
所有緩衝使用 aligned(4) 保持 DMA 傳輸效率。
八、擴充建議
抽象化 Codec：改為函式指標表支持不同型號。
增加配置檔管理：Flash 儲存多組 I2S + Codec Profile，可由 GUI 切換。
增加 DSP 範例（EQ/RMS/AGC）放入 ping-pong 處理階段驗證效能。
加入 Loopback 測試模式（ADC→USB→Host→USB→DAC）快速驗證整鏈可靠性。
加入更細緻錯誤碼（分：I2C_Timeout / I2C_NACK / HID_Frame_Error）。
導入 CRC 校驗 HID 大型批次資料（降低傳輸錯誤風險）。
九、關鍵術語速覽
HID 分幀：每 64 bytes 資料包 + FrameNum 最高位 (0x80) 表示尚有後續。
Burst Mode：一次寫入大量連續暫存器資料，提高配置效率。
Ping-Pong Buffer：兩組交替供 PDMA/I2S 使用，降低等待時間。
Adaptive UAC：在特殊通道與取樣率組合下自調整緩衝策略。
ZC (Zero-Cross)：避免音量跳變產生爆音。
十、最精簡一句話總結
這是整合 USB 音訊 + HID 遠端控制 + I2C Codec 配置 + I2S/PDMA 高效音訊傳輸的 NAU88C22 評估與調試韌體平台。
