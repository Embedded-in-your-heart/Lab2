# STM32 WiFi Sensor Dashboard

基於 STM32 B-L475E-IOT01A Discovery Board，透過 WiFi (TCP) 將感測器資料傳送至電腦端，並以 Dash + Plotly 即時視覺化。

## 功能

### Basic (85 pts)
- 讀取 LSM6DSL 3 軸加速度計（mg）與 3 軸陀螺儀（mdps）
- 透過 WiFi TCP 協定將感測器資料以 JSON 格式發送至主機
- Python Dash Dashboard 即時顯示波形圖與數值

### Option 1 (5 pts)
- 啟用 LSM6DSL Significant Motion Detection（顯著運動偵測）
- LSM6DSL 偵測到顯著運動時透過 INT1 (PD11) 產生 GPIO EXTI 中斷
- Dashboard 上以黃色垂直線標記事件，底部顯示事件日誌

## 架構

```
STM32 B-L475E-IOT01A                         Host (Mac/Linux/Windows)
┌─────────────────────┐                      ┌──────────────────────┐
│  LSM6DSL (I2C)      │                      │  python_server/      │
│  ├─ Accelerometer   │    WiFi TCP          │  ├─ server.py        │
│  ├─ Gyroscope       │ ──────────────────>  │  │  (TCP Server)     │
│  └─ Sig. Motion INT1│    JSON data         │  └─ Dash Dashboard   │
│                     │                      │     (localhost:8050)  │
│  TIM2 (100ms)       │                      └──────────────────────┘
│  es-WiFi (SPI3)     │
└─────────────────────┘
```

## JSON 資料格式

```json
{"acc":{"x":12,"y":-5,"z":1002},"gyro":{"x":0.35,"y":-1.20,"z":0.05},"sig_motion":0}
```

| 欄位 | 來源 | 單位 | 說明 |
|------|------|------|------|
| `acc.x/y/z` | `BSP_ACCELERO_AccGetXYZ()` | mg | 3 軸加速度 |
| `gyro.x/y/z` | `BSP_GYRO_GetXYZ()` | mdps | 3 軸角速度 |
| `sig_motion` | LSM6DSL INT1 EXTI 中斷 | 0 / 1 | 顯著運動偵測 |

## STM32 韌體設定

### WiFi 連線參數（`Src/main.c`）

```c
#define SSID     "IMCC"
#define PASSWORD "i<3ntuim"
uint8_t RemoteIP[] = {192,168,88,74};
#define RemotePORT 25550
```

使用前請修改 `SSID`、`PASSWORD`、`RemoteIP` 為你的網路環境。

### 硬體資源

| 資源 | 用途 |
|------|------|
| I2C2 | LSM6DSL 感測器通訊 |
| SPI3 | es-WiFi 模組通訊 |
| TIM2 | 100ms 週期中斷（感測器讀取計時） |
| EXTI1 (PE1) | WiFi 模組 data ready |
| EXTI11 (PD11) | LSM6DSL INT1（Significant Motion） |
| USART1 | 115200 baud debug 輸出 |
| LED2 (PB14) | Significant Motion 發生時閃爍 |

### Timer 設定

```
系統時鐘 = 80MHz
Prescaler = 7999 → Timer clock = 10kHz
Period = 999 → 溢位間隔 = 100ms
```

TIM2 中斷設定 `sensor_timer_flag`，主迴圈檢查 flag 後讀取感測器並送出資料，取代 `HAL_Delay()` 忙等。

### Significant Motion 偵測

透過 I2C 直接寫入 LSM6DSL 暫存器啟用：

| 暫存器 | 位元 | 作用 |
|--------|------|------|
| `CTRL10_C (0x19)` | FUNC_EN, PEDO_EN, SIGN_MOTION_EN | 啟用嵌入式功能與顯著運動偵測 |
| `MD1_CFG (0x5E)` | INT1_SIGN_MOT | 路由事件至 INT1 腳位 |

LSM6DSL 內部基於計步器演算法判定是否為「真正的人體移動」，非單純加速度門檻值。

## Python Dashboard 使用方式

### 環境需求
- Python >= 3.11
- [uv](https://docs.astral.sh/uv/) 套件管理工具

### 啟動

```bash
cd python_server
uv sync
uv run python server.py
```

瀏覽器開啟 http://localhost:8050

### 命令列選項

```
--host       TCP 綁定位址（預設 0.0.0.0）
--port       TCP 埠號（預設 25550）
--dash-port  Dashboard 埠號（預設 8050）
```

### Dashboard 畫面

- **狀態列**：連線狀態、STM32 IP、累計樣本數
- **數值卡片**：加速度與陀螺儀最新數值
- **上方圖表**：加速度計 X/Y/Z 即時波形
- **下方圖表**：陀螺儀 X/Y/Z 即時波形（虛線）
- **黃色垂直線**：Significant Motion 事件標記
- **事件日誌**：Significant Motion 發生時間戳記

## 專案結構

```
WiFi_Client_Server/
├── Inc/                        # STM32 headers
│   └── main.h
├── Src/                        # STM32 application
│   ├── main.c                  # 主程式（感測器讀取、WiFi、TIM2、中斷）
│   ├── stm32l4xx_it.c          # 中斷處理（EXTI1, EXTI11, TIM2）
│   └── system_stm32l4xx.c
├── Common/                     # WiFi middleware (es-WiFi driver)
├── Drivers/
│   ├── BSP/B-L475E-IOT01/     # Board Support Package
│   ├── BSP/Components/lsm6dsl/ # LSM6DSL sensor driver
│   ├── CMSIS/
│   └── STM32L4xx_HAL_Driver/
├── python_server/              # Python Dashboard
│   ├── server.py               # TCP server + Dash 視覺化
│   └── pyproject.toml          # uv 依賴管理
├── STM32CubeIDE/               # IDE 專案檔
└── README.md
```

## 編譯與燒錄

1. 用 STM32CubeIDE 開啟 `STM32CubeIDE/.project`
2. 修改 `Src/main.c` 中的 WiFi 參數
3. Build Project → Run → 燒錄至板子
4. 開啟電腦端 Python Dashboard
5. 板子上電後自動連線並開始傳送資料
