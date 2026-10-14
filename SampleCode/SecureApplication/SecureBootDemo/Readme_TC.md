# Secure Boot Demo 範例程式

> **本文件由 AI 自動生成，內容僅供參考。**  
> * 請務必驗證所有技術細節與程式碼的正確性
> * 安全性相關實作（如加密演算法、金鑰管理）需進行完整測試

## 描述
本範例程式碼示範如何在 NuMicro M3351 系列上實作安全啟動 (Secure Boot) 流程。它建立了一個受信任的啟動鏈，由 Boot Loader (NuBL2) 在執行前驗證 Secure World 韌體 (NuBL32) 與 Non-Secure World 韌體 (NuBL33) 的合法性。

## 專案結構
*   `NuBL2/`: 第二階段引導程式 (Root of Trust) 原始碼。
*   `NuBL32/`: Secure World 應用程式原始碼。
*   `NuBL33/`: Non-Secure World 應用程式原始碼。
*   `FwSign.py`: 用於簽署韌體映像檔的 Python 腳本。

## 運作流程
1.  **系統初始化**: NuBL2 初始化系統時脈與加密硬體加速器 (Crypto)。
2.  **安全檢查**: 檢查 SOTP (Secure One-Time Programmable) 狀態以確認裝置是否處於安全鎖定狀態。
3.  **韌體驗證**:
    *   **完整性檢查**: 計算 NuBL32 與 NuBL33 映像檔的 SHA-256 雜湊值，並與儲存在 TLV (Type-Length-Value) 區域中的雜湊值進行比對。
    *   **真實性檢查**: 使用內建的公鑰 (Public Key) 驗證映像檔的 ECDSA-P256 數位簽章。
4.  **安全隱藏 (Secure Conceal)**: 可選擇性配置並啟用 Secure Conceal 功能，將 NuBL2 的程式碼與數據隱藏，防止後續執行的應用程式存取。
5.  **啟動**: 跳轉至 NuBL32 的進入點開始執行。

## 使用方法
1.  編譯 `NuBL2`、`NuBL32` 與 `NuBL33` 專案。
2.  使用提供的 `FwSign` 腳本對 `NuBL32` 與 `NuBL33` 的二進位檔進行簽章。
3.  將 `NuBL2` 以及簽章後的 `NuBL32`/`NuBL33` 映像檔下載至目標裝置。
4.  重置裝置。NuBL2 將開始執行，驗證映像檔，並啟動應用程式。

## 關鍵功能
*   **硬體加速**: 使用 M3351 Crypto 模組進行快速的 SHA-256 計算與 ECC 簽章驗證。
*   **安全隱藏**: 保護 Bootloader 本身不被應用程式存取或竄改。
*   **MCUboot 相容**: 使用與 MCUboot 相容的映像檔標頭 (Header) 與 TLV 格式。
