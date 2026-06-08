# 01studio pyDrone ESP32-S3 移植到官方 MicroPython 教學

## 目標

把 [01studio-lab/micropython](https://github.com/01studio-lab/micropython) fork 中的
pyDrone 板子（ESP32-S3）移植到最新官方 [micropython/micropython](https://github.com/micropython/micropython)，
在 ESP-IDF 5.3.2 環境下成功建置並飛行。

---

## 一、環境準備

### 1-1 主機需求

| 項目 | 版本 |
|------|------|
| OS   | Windows 11（或任何可執行 Docker 的系統）|
| Docker Desktop | 最新版 |
| Git  | 任意版本 |

### 1-2 Clone 官方 MicroPython

```bash
git clone https://github.com/micropython/micropython.git mp-new
cd mp-new
```

### 1-3 拉取 IDF 5.3.2 Docker image

```bash
docker pull espressif/idf:v5.3.2
```

> 這個 image 包含整個 ESP-IDF toolchain，不需要在主機安裝任何 IDF 工具。

### 1-4 準備舊 fork 的原始碼（參考用）

```bash
git clone https://github.com/01studio-lab/micropython.git mp-old
```

---

## 二、移植範圍說明

舊 fork 在官方 MicroPython 之上新增了以下東西：

| 元件 | 位置 | 說明 |
|------|------|------|
| PYDRONE board definition | `ports/esp32/boards/PYDRONE/` | 板子 cmake/header/partition/sdkconfig |
| py-drone C modules | `ports/esp32/py-drone/` | 飛控演算法、感測器驅動、MicroPython bindings |
| sdkconfig 片段 | `ports/esp32/boards/sdkconfig.cam` `sdkconfig.usb` | 攝像頭與 USB-OTG 設定 |
| cmake 整合 | `ports/esp32/esp32_common.cmake` | 把 py-drone 接入 IDF CMake 系統 |

---

## 三、複製 PYDRONE Board 定義

從舊 fork 把 board 目錄複製過來：

```bash
cp -r mp-old/ports/esp32/boards/PYDRONE  mp-new/ports/esp32/boards/
cp mp-old/ports/esp32/boards/sdkconfig.cam  mp-new/ports/esp32/boards/
cp mp-old/ports/esp32/boards/sdkconfig.usb  mp-new/ports/esp32/boards/
cp -r mp-old/ports/esp32/py-drone  mp-new/ports/esp32/
```

---

## 四、修改 `mpconfigboard.cmake`

這是移植最關鍵的一個檔案：`ports/esp32/boards/PYDRONE/mpconfigboard.cmake`。

舊 fork 的寫法是直接列出所有 sdkconfig 檔案，**不包含** 官方的 `sdkconfig.base`，
會缺少 `CONFIG_MBEDTLS_PLATFORM_TIME_ALT=y` 等必要設定，導致編譯錯誤。

**正確寫法（以官方 S3 base 為底）：**

```cmake
# 繼承官方 ESP32-S3 共用設定（包含 sdkconfig.base 必要項）
include(${MICROPY_PORT_DIR}/boards/mpconfigboard_esp32s3_common.cmake)

# 再疊加 PYDRONE 特有的 sdkconfig（後面的覆蓋前面的）
list(APPEND SDKCONFIG_DEFAULTS
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)

# 啟用 py-drone C modules
set(MICROPY_PORT_DRONE "y")

# 停用 ULP（IDF 5.x header 位置已搬移）
set(MICROPY_PY_ESP32_ULP 0)

# 補上 IDF 5.x 的 ULP include 路徑（即使停用仍需讓 esp32_ulp.c 編譯過）
list(APPEND IDF_COMPONENTS ulp)
target_include_directories(${IDF_TARGET}.elf PRIVATE
    /opt/esp/idf/components/ulp/ulp_fsm/include
    /opt/esp/idf/components/ulp/ulp_fsm/include/esp32s3
)
```

> **原則：永遠用 `list(APPEND SDKCONFIG_DEFAULTS ...)` 疊加，不要整個覆蓋。**

---

## 五、修復 `sdkconfig.board` 的相容性問題

舊 fork 的 `sdkconfig.board` 有兩類問題：

### 5-1 IDF config key 更名

| 舊 key（IDF 4.x / 01studio）| 新 key（IDF 5.x）|
|------|------|
| `CONFIG_ESP32S3_ULP_COPROC_ENABLED` | `CONFIG_ULP_COPROC_ENABLED` |

直接把舊的 key 改掉即可（更名只產生 warning，但最好修乾淨）。

### 5-2 Fork 專屬 hook 被移除

舊 fork 在 `mpthreadport.c` 裡定義了 `vPortCleanUpTCB()`，
對應 `CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP=y`。

官方 MicroPython **沒有** 這個函數，所以：

```
undefined reference to vPortCleanUpTCB
```

**修法：** 從 `sdkconfig.board` 移除這一行：

```ini
# 刪除這行
CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP=y
```

官方的 `sdkconfig.base` 已經設定為 `n`，不需要再加。

---

## 六、整合 py-drone 到 CMake 建置系統

編輯 `ports/esp32/esp32_common.cmake`，在 `# Provide the default LD fragment` 之前加入：

```cmake
# py-drone C modules (PYDRONE board)
if(MICROPY_PORT_DRONE STREQUAL "y")
    set(DRONE_DIR ${MICROPY_PORT_DIR}/py-drone)
    file(GLOB_RECURSE MICROPY_SOURCE_DRONE "${DRONE_DIR}/*.c")
    list(APPEND MICROPY_SOURCE_BOARD ${MICROPY_SOURCE_DRONE})
    list(APPEND MICROPY_SOURCE_QSTR  ${MICROPY_SOURCE_DRONE})
    list(APPEND MICROPY_INC_DRONE
        ${DRONE_DIR}
        ${DRONE_DIR}/drivers/i2c_bus/include
        ${DRONE_DIR}/drivers/i2c_devices/mpu6050/include
        ${DRONE_DIR}/drivers/i2c_devices/spl06/include
        ${DRONE_DIR}/drivers/i2c_devices/hmc5883l/include
        ${DRONE_DIR}/drivers/motors/include
        ${DRONE_DIR}/drivers/pm/include
        ${DRONE_DIR}/drivers/led/include
        ${DRONE_DIR}/mpmodules
        ${DRONE_DIR}/port
        ${DRONE_DIR}/dsp_lib/include
        ${DRONE_DIR}/utils/interface
    )
endif()
```

然後在 `idf_component_register(... INCLUDE_DIRS ...)` 的 include 清單加上：

```cmake
${MICROPY_INC_DRONE}
```

---

## 七、Partition Table

pyDrone 的 drone C modules 很大（~1 MB），官方預設的 app partition（`0x190000` = 1.5625 MB）
在加入所有模組後只剩 `0x7a0` bytes（約 2 KB），不夠用。

`ports/esp32/boards/PYDRONE/partitions-8MiB.csv` 已針對 8 MiB flash 調整好，
確認 `mpconfigboard.cmake` 有 `set(MICROPY_BOARD_FLASH_SIZE "8MiB")` 及正確引用此檔案。

---

## 八、建置

所有 ESP32 build 在 Docker 容器內執行：

```bash
# 1. 啟動 IDF 5.3.2 容器，掛載 repo
docker run --rm -it \
  -v "C:/github/mp-new:/micropython" \
  -w /micropython/ports/esp32 \
  espressif/idf:v5.3.2 bash

# 2. 容器內：初始化 submodules（第一次）
make submodules

# 3. 建置 PYDRONE
rm -rf build-PYDRONE
make BOARD=PYDRONE -j$(nproc)
```

成功輸出的最後幾行：

```
...
esptool.py v4.x.x
Merged 2 ELF sections
Generated /micropython/ports/esp32/build-PYDRONE/firmware.bin
```

---

## 九、燒錄

連接 pyDrone 開發板後（USB-OTG 或 UART）：

```bash
# 在 Docker 容器內，或主機上安裝 esptool
esptool.py --chip esp32s3 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 \
  write_flash 0x0 build-PYDRONE/firmware.bin
```

Windows 上 COM 埠：

```bash
make BOARD=PYDRONE PORT=COM3 deploy
```

---

## 十、常見錯誤與解法

### `undefined reference to vPortCleanUpTCB`

`sdkconfig.board` 殘留 `CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP=y`，刪除即可。

### `fatal error: esp32s3/ulp.h: No such file`

IDF 5.x 把 ULP header 搬到 `ulp/ulp_fsm/include/esp32s3/`。
在 `mpconfigboard.cmake` 補上 ULP include path（見第四節）。
或直接 `set(MICROPY_PY_ESP32_ULP 0)` 停用 ULP。

### `CONFIG_MBEDTLS_PLATFORM_TIME_ALT` 相關錯誤

`mpconfigboard.cmake` 沒有 `include(mpconfigboard_esp32s3_common.cmake)`，
導致 `sdkconfig.base` 沒被載入。確認 cmake 有繼承官方 S3 base config。

### App partition too small

加入 py-drone 模組後 partition 不夠，修改 `partitions-8MiB.csv` 放大 app partition。
每次改 partition table 都要全 clean：`rm -rf build-PYDRONE`。

### sdkconfig 更名警告

```
warning: esp32s3_ulp_coproc_enabled (defined at ...): Deprecated, ...
```

把 `sdkconfig.board` 裡的舊 key 改為新名稱即可消除。

---

## 十一、Repository 管理建議

本移植的 Git 結構：

```
micropython/micropython  (upstream，官方)
    ↓  clone
local: C:\github\mp-new
    ├── remote origin  → micropython/micropython  (追蹤官方)
    ├── remote myfork  → pc0808f/micropython      (你的 GitHub)
    ├── branch master          (與 upstream 同步)
    └── branch pydrone/esp32s3-port  (移植工作)
```

### 追蹤官方更新

```bash
# 拉取官方最新
git fetch origin
git checkout master
git merge origin/master

# 把移植 branch rebase 到最新 master
git checkout pydrone/esp32s3-port
git rebase master
```

### Push 到自己的 GitHub

```bash
git push myfork master pydrone/esp32s3-port
```

---

## 十二、後續待辦

- [ ] 攝像頭模組移植（`MICROPY_PORT_CAMLIB`）— 需補 camera C module
- [ ] Web stream 移植（`MICROPY_PORT_WEB_STREAM`）
- [ ] 調整飄移修正參數（角度環 ki、trim 軸偏移）
- [ ] 驗證 OTA 更新流程
