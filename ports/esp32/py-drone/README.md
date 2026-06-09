# pyDrone 飛行控制調參手冊

## 目錄

1. [系統架構概覽](#1-系統架構概覽)
2. [基本飛行 API](#2-基本飛行-api)
3. [狀態讀取](#3-狀態讀取)
4. [可調參數總覽](#4-可調參數總覽)
5. [參數調整 API](#5-參數調整-api)
6. [thrustBase 懸停油門](#6-thrustbase-懸停油門)
7. [垂直高度 PID（pidVZ / pidZ）](#7-垂直高度-pidpidvz--pidz)
8. [姿態角度 PID（pidAngle）](#8-姿態角度-pidpidangle)
9. [常見問題診斷與調整建議](#9-常見問題診斷與調整建議)
10. [完整調參流程範例](#10-完整調參流程範例)
11. [新功能測試流程](#11-新功能測試流程)

---

## 1. 系統架構概覽

```
用戶油門輸入
     ↓
[指令解析] setpoint.position.z = 目標高度 (cm)
     ↓
[pidZ] 位置環  kp=6 kd=4.5
  誤差 = 目標高度 - 估測高度  →  輸出：目標垂直速度 (最大±120 cm/s)
     ↓
[pidVZ] 速度環  kp=100 ki=60 kd=10  iLimit=200
  誤差 = 目標速度 - 實際速度  →  輸出：推力補償量 (±40000)
     ↓
實際推力 = thrustBase + pidVZ輸出  (限制在 1000~60000)
     ↓
四個馬達 PWM

高度估測來源：
  氣壓計 (cm) + 加速度計積分
  起飛後 0.5 秒氣壓計無效，純靠加速度計
```

姿態控制是獨立的雙環：

```
[pidAngle] 角度環 → 輸出目標角速度
     ↓
[pidRate]  角速度環 → 輸出 roll/pitch/yaw 控制量
```

---

## 2. 基本飛行 API

```python
from espdrone import drone

d = drone()          # 初始化飛控系統
d = drone(debug=1)   # 開啟除錯輸出

d.take_off()              # 起飛到預設高度 80 cm
d.take_off(distance=120)  # 起飛到 120 cm

d.control(rol=0, pit=0, yaw=0, thr=0)
# rol: 左右傾斜  -100 ~ +100  (×0.1 → ±10°)
# pit: 前後傾斜  -100 ~ +100  (×0.1 → ±10°)
# yaw: 轉向      -100 ~ +100  (×2 → ±200°/s)
# thr: 油門微調  -100 ~ +100  (相對於定高基準)

d.trim(rol=0, pit=0)   # 微調靜態偏移 (×0.01 → ±1°)
d.landing()            # 緩降落地
d.stop()               # 緊急停機（直接斷油門）
```

---

## 3. 狀態讀取

```python
states = d.read_states()
# 回傳 10 個數值的 tuple：
# [0] roll×100      實際滾轉角 (°×100，例如 -532 = -5.32°)
# [1] pitch×100     實際俯仰角
# [2] yaw×100       實際偏航角
# [3] cmd_roll×100  指令滾轉
# [4] cmd_pitch×100 指令俯仰
# [5] cmd_yaw×100   指令偏航
# [6] thrust%       油門百分比 (0~100)
# [7] battery×100   電池電壓 (例如 380 = 3.80V)
# [8] height_cm     估測高度 cm (起飛點為0)
# [9] althold_thrust 定高 PID 學習到的實際懸停推力

# 讀取感測器
d.read_accelerometer()  # → (gyro_x, gyro_y, gyro_z, acc_y, acc_x, acc_z)
d.read_air_pressure()   # → (pressure×100, temperature×100)
```

---

## 4. 可調參數總覽

### 在 configParam 結構中（有記憶體持久化）

| 參數 | 當前預設值 | 單位 | 說明 |
|---|---|---|---|
| **thrustBase** | 34000 | PWM | 懸停基礎油門（0~65535，硬體限60000） |
| **pidVZ.kp** | 100 | — | 垂直速度環比例增益 |
| **pidVZ.ki** | 60 | — | 垂直速度環積分增益 |
| **pidVZ.kd** | 10 | — | 垂直速度環微分增益 |
| pidZ.kp | 6 | — | 高度位置環比例增益 |
| pidZ.kd | 4.5 | — | 高度位置環微分增益 |
| pidAngle.roll.kp | 8 | — | 角度環滾轉比例 |
| pidAngle.roll.ki | 0.8 | — | 角度環滾轉積分（補償偏移） |
| pidAngle.pitch.kp | 8 | — | 角度環俯仰比例 |
| pidAngle.pitch.ki | 0.8 | — | 角度環俯仰積分 |
| pidAngle.yaw.kp | 20 | — | 角度環偏航比例 |
| pidAngle.yaw.kd | 1.5 | — | 角度環偏航微分 |
| pidRate.roll.kp | 300 | — | 角速度環滾轉比例（內環，一般不調） |
| pidRate.pitch.kp | 300 | — | 角速度環俯仰比例 |
| pidRate.yaw.kp | 200 | — | 角速度環偏航比例 |
| pidRate.yaw.ki | 18.5 | — | 角速度環偏航積分 |

### 運行時可調（Python API，非 configParam）

| 參數 | 預設值 | 說明 |
|---|---|---|
| Mahony Kp | 0.4 | 姿態融合比例增益；調低可減少馬達振動對姿態的污染 |
| Mahony Ki | 0.001 | 姿態融合積分增益；調高可加快陀螺零偏校正速度 |

### 硬編碼（需修改 C 原始碼）

| 參數 | 位置 | 當前值 | 說明 |
|---|---|---|---|
| pidVZ iLimit | position_pid.c | 200 | 積分值上限，ki×iLimit = 最大I項輸出 |
| wBaro | state_estimator.c | 0.35 | 氣壓計融合基礎權重（飛快時自動降低） |
| 氣壓暖機時間 | state_estimator.c | 0.5 秒 | 起飛後氣壓計遮蔽時間 |
| Z 速度衰減 | state_estimator.c | 0.995/步 | 防止加速計噪聲積分漂移，時間常數約 1.6 秒 |
| 動態氣壓權重 | state_estimator.c | 自動 | 垂直速度 >30 cm/s 時降低氣壓權重，抑制超衝 |
| pidZ 速度限幅 | position_pid.c | ±120 cm/s | 高度環輸出上限 |

---

## 5. 參數調整 API

```python
# thrustBase
d.set_thrust_base(34000)   # 設定懸停基礎油門
d.get_thrust_base()        # 讀取當前設定值

# 垂直速度環（最常調整）
d.set_pid_vz(kp=100, ki=60, kd=10)
d.get_pid_vz()             # → (kp, ki, kd)

# 高度位置環
d.set_pid_z(kp=6, kd=4.5)
d.get_pid_z()              # → (kp, kd)

# 姿態角度環
d.set_pid_angle(axis=0, kp=8, ki=0.8, kd=0)  # 0=roll
d.set_pid_angle(axis=1, kp=8, ki=0.8, kd=0)  # 1=pitch
d.set_pid_angle(axis=2, kp=20, ki=0, kd=1.5) # 2=yaw

# Mahony 姿態融合增益
d.get_mahony()            # → (0.4, 0.001)  讀取目前 Kp, Ki
d.set_mahony(0.3, 0.001)  # 降低 Kp，減少馬達振動污染姿態估測
d.set_mahony(0.4, 0.002)  # 提高 Ki，加快陀螺零偏校正速度
```

> **注意**：所有 `set_*` 會即時生效，下一個控制週期（4ms）就套用新值，飛行中修改也有效。

---

## 6. thrustBase 懸停油門

### 什麼是 thrustBase

`thrustBase` 是定高 PID 計算推力時加上去的基礎值：

```
實際推力 = thrustBase + pidVZ 輸出
```

當飛機穩定懸停時，pidVZ 輸出應趨近於 0，此時 `thrustBase` = 實際懸停推力。

### 為什麼重要

- `thrustBase` 太低：PID 需要輸出大量正 I 項才能維持高度，積分容易飽和，反應變慢
- `thrustBase` 太高：PID 需要輸出大量負 I 項，同樣有積分問題，且起飛時會過衝

### 如何找到正確值

1. 讓飛機穩定懸停 30 秒以上
2. 讀取 `read_states()[9]` — 這是系統自動學習的低通平滑推力值
3. 用學習值更新設定：

```python
# 懸停後讀取學習值
states = d.read_states()
learned = states[9]
print(f"學習到的懸停推力: {learned}")

# 如果與設定值差距 > 2000，更新設定
current = d.get_thrust_base()
if abs(learned - current) > 2000:
    d.set_thrust_base(int(learned))
    print(f"已更新 thrustBase: {int(learned)}")
```

> 系統在穩定飛行 4 秒後會自動學習並更新 `thrustBase`（每 4 秒更新一次，差距 > 1000 才更新）。

---

## 7. 垂直高度 PID（pidVZ / pidZ）

### 控制鏈說明

```
高度誤差(cm) → [pidZ kp=6 kd=4.5] → 目標速度(cm/s) → [pidVZ kp=100 ki=60 kd=10] → 推力補償
```

### pidVZ 積分的作用

積分項（I）負責補償長期穩態誤差：若飛機持續低於目標高度，I 項會慢慢累積正值來補充推力。

**積分限幅（iLimit）的意義**：

```
最大 I 項輸出 = ki × iLimit = 60 × 200 = 12000
```

如果 `thrustBase` 準確，懸停時 P+D ≈ 0，I 項也趨近 0。只有在 `thrustBase` 偏離時，I 項才需要補償。

### 參數調整原則

**pidVZ.ki（積分增益）**
- 太大（原本 150）：積分累積過快，超過目標高度後難以消退 → 飛到天花板
- 太小（< 30）：無法補償 thrustBase 誤差 → 定高時緩慢下沉
- 建議從 60 開始，若穩定下沉可調到 80，若超衝再降到 40

**pidVZ.kp（比例增益）**
- 太大：高度變化時推力響應過激，導致振盪
- 太小：對高度誤差反應遲鈍
- 通常不需要調整，保持 100

**pidZ.kp（位置環比例）**
- 控制「高度誤差換算成速度指令」的速度
- 調大：快速追蹤目標高度，但可能過衝
- 調小：緩慢平穩，但定高精度差

---

## 8. 姿態角度 PID（pidAngle）

### ki（積分）的作用

`pidAngle.roll.ki` 和 `pidAngle.pitch.ki` 用來補償物理偏差：

- IMU 安裝角度不正
- 機體重心偏移
- 左右馬達推力不對稱

**預設值 ki = 0.8** 是針對一般個體的折衷值。

### 調整時機

| 現象 | 可能原因 | 建議調整 |
|---|---|---|
| 懸停時持續往同一方向漂移 | roll/pitch ki 太小 | 調高 ki（0.8 → 1.2） |
| 懸停時緩慢左右搖擺（低頻） | roll/pitch ki 太大 | 調低 ki（0.8 → 0.4） |
| 懸停時快速振盪（高頻） | roll/pitch kp 太大 | 調低 kp（8 → 6） |
| 偏航方向轉動不準 | yaw kp 太小 | 調高 yaw kp（20 → 25） |

---

## 9. 常見問題診斷與調整建議

### ❶ 飛到天花板，油門減到底也不下來

**原因**：pidVZ 積分項（I）過大，超過 P 項的反向力。

**計算驗證**：
```
P 項 = kp × 速度誤差 = 100 × (-120) = -12000
I 項（飽和） = ki × iLimit = 60 × 200 = +12000
合計推力 = -12000 + 12000 + 34000 = 34000 → 仍在懸停！
```
當 I 項飽和，P 項被完全抵消，飛機不回應油門。

**解決方法**（按順序排查）：
1. 確認 `thrustBase` 正確（最重要）：`d.get_thrust_base()` vs `read_states()[9]`
2. 降低 `ki`：`d.set_pid_vz(100, 40, 10)`
3. 若問題仍存在，修改 C 原始碼中 `iLimit`（position_pid.c:39）從 200 降到 100

### ❷ 定高時緩慢下沉

**原因**：`thrustBase` 低於實際懸停推力，I 項補償不足。

**診斷**：
```python
states = d.read_states()
learned = states[9]
current = d.get_thrust_base()
print(f"學習值: {learned}, 設定值: {current}, 差距: {learned - current}")
```

**解決方法**：
1. 先讓飛機懸停 30 秒讓學習值穩定
2. 用學習值更新 thrustBase：`d.set_thrust_base(int(learned))`
3. 或提高 ki：`d.set_pid_vz(100, 80, 10)`（治標）

### ❸ 起飛超衝，飛到目標高度後繼續往上一段距離

**原因**：起飛時積分快速累積，到達目標高度時 I 項仍有殘餘正值。

**解決方法**：
1. 降低 ki：`d.set_pid_vz(100, 40, 10)`
2. 或提高 pidZ.kd 讓速度環提早減速：`d.set_pid_z(6, 6.0)`

### ❹ 懸停時持續往左右或前後漂移

**原因**：重心偏移或 IMU 傾斜，角度環 ki 不足以補償。

**解決方法**：
```python
# 先用 trim 做粗調（緊急補偏）
d.trim(rol=-30, pit=0)   # 往左漂移 → 給正值

# 若漂移固定，提高角度環 ki（精細補償）
d.set_pid_angle(0, 8, 1.2, 0)  # roll ki: 0.8 → 1.2
d.set_pid_angle(1, 8, 1.2, 0)  # pitch ki: 0.8 → 1.2
```

### ❺ 姿態不穩，有高頻振盪（嗡嗡聲）

**原因**：角度環或角速度環 kp 過高。

**解決方法**：
```python
# 先降低角度環 kp
d.set_pid_angle(0, 6, 0.8, 0)   # roll kp: 8 → 6
d.set_pid_angle(1, 6, 0.8, 0)   # pitch kp: 8 → 6

# 若仍振盪，角速度環 kp 需在 C 原始碼中調整（config_param.c）
```

### ❻ 高度估測跳動，定高不穩

**原因**：氣壓計受螺旋槳氣流影響。

**現有保護**：起飛後 0.5 秒內氣壓計輸出被屏蔽（0.5 秒暖機期）。

**若暖機後仍跳動**：
- 確認飛機外殼密封性（氣流進入感測器腔室）
- 硬體解法優先，軟體無法完全補救

### ❼ 旋轉（yaw）方向不穩或慢慢偏轉

**原因**：yaw 角速度環 ki 補償不足。

**調整**：
```python
d.set_pid_angle(2, 20, 0, 1.5)  # yaw 調整 kd 先試試
# yaw ki 是在 pidRate（角速度環），需修改 config_param.c
```

---

## 10. 完整調參流程範例

```python
from espdrone import drone
import time

d = drone()

# === 步驟 1：確認 thrustBase ===
print("當前 thrustBase:", d.get_thrust_base())

# 起飛，懸停 30 秒觀察
d.take_off(distance=80)
time.sleep(30)

states = d.read_states()
height = states[8]
learned_thrust = states[9]
print(f"高度: {height} cm, 學習推力: {learned_thrust}")

# 若高度穩定（±5cm）且推力與設定差距 > 2000，更新
if abs(height - 80) < 10:
    d.set_thrust_base(int(learned_thrust))
    print(f"thrustBase 更新為 {int(learned_thrust)}")

# === 步驟 2：觀察漂移 ===
# 懸停中觀察 states[0]（roll）和 states[1]（pitch）是否持續偏向某側
# 若 roll 持續 > +200（+2°），代表往右漂
d.set_pid_angle(0, 8, 1.2, 0)  # 提高 roll ki

# === 步驟 3：確認定高反應 ===
# 若緩慢下沉
d.set_pid_vz(100, 80, 10)   # 提高 ki

# 若超衝到天花板
d.set_pid_vz(100, 40, 10)   # 降低 ki

# === 步驟 4：讀取確認當前設定 ===
print("pidVZ:", d.get_pid_vz())
print("pidZ:", d.get_pid_z())
print("thrustBase:", d.get_thrust_base())

d.landing()
```

---

---

## 11. 新功能測試流程

本節針對本次從 Crazyflie 移植的三項改動提供驗證步驟：
**速度衰減**、**Mahony 參數化**、**動態氣壓計權重**。

速度衰減和動態氣壓權重是被動的，不需要主動調數字，飛完對比行為差異即可。
Mahony Kp/Ki 需要根據實際飛行結果微調。

---

### 步驟一：地面靜置確認（起飛前）

```python
from espdrone import drone
import time

d = drone()

# 確認預設值正確
print("Mahony:", d.get_mahony())        # 應為 (0.4, 0.001)
print("pidVZ:", d.get_pid_vz())         # 應為 (100.0, 60.0, 10.0)
print("thrustBase:", d.get_thrust_base()) # 應為 34000

# 靜置 10 秒觀察姿態穩定性
for i in range(10):
    s = d.read_states()
    print(f"roll={s[0]/100:.2f}°  pitch={s[1]/100:.2f}°  yaw={s[2]/100:.2f}°")
    time.sleep(1)
```

**判斷**：
- roll/pitch 在 ±1° 內小幅跳動 → 正常
- 靜置就持續單向漂移 → Mahony Ki 太小，試 `d.set_mahony(0.4, 0.002)`

---

### 步驟二：起飛超衝觀察（驗證動態氣壓權重）

```python
d.take_off(distance=100)

# 起飛後前 5 秒密集取樣
for i in range(50):
    s = d.read_states()
    print(f"t={i*0.1:.1f}s  高度={s[8]}cm")
    time.sleep(0.1)

d.landing()
```

**判斷**：
- 高度最高峰值距目標 100cm 的超衝量（越小越好）
- 改動前若衝到 140cm，改動後預期 ≤ 120cm

---

### 步驟三：長時懸停觀察（驗證速度衰減）

```python
d.take_off(distance=80)
time.sleep(3)  # 等穩定

for i in range(120):
    s = d.read_states()
    if i % 15 == 0:
        print(f"t={i:3d}s  高度={s[8]:4d}cm  "
              f"roll={s[0]/100:.1f}°  pitch={s[1]/100:.1f}°  "
              f"推力學習值={s[9]}")
    time.sleep(1)

# 懸停 2 分鐘後讀取學習推力
s = d.read_states()
print(f"\n學習推力: {s[9]}  → 建議 set_thrust_base({int(s[9])})")
d.landing()
```

**判斷**：
- 高度在 2 分鐘內是否維持在 ±10cm → 速度衰減有效
- 推力學習值趨近穩定後就是真實的 `thrustBase`

---

### 步驟四：Mahony 調參（根據實際現象）

| 現象 | 建議調整 | 指令 |
|---|---|---|
| 姿態讀值高頻跳動（>±0.5°） | Kp 降低 | `d.set_mahony(0.3, 0.001)` |
| 長時間懸停姿態累積偏移 >2° | Ki 提高 | `d.set_mahony(0.4, 0.002)` |
| 目前正常 | 不動 | — |

```python
# Kp/Ki 調整後即時生效，不需重啟
d.set_mahony(0.3, 0.001)
print("新 Mahony 值:", d.get_mahony())
```

---

### 調參優先順序

```
1. thrustBase   ← 每次換電池都要確認，差距 >2000 就更新
2. pidVZ ki     ← 觀察是否超衝（降 ki）或下沉（升 ki）
3. Mahony Kp    ← 觀察靜態姿態讀值跳動幅度，跳動大就降
4. Mahony Ki    ← 觀察長時間懸停的姿態累積漂移，漂移大就升
```

---

## 參數速查卡

| 症狀 | 首先調整 | 方向 |
|---|---|---|
| 飛到天花板不下來 | `thrustBase` 或 `pidVZ.ki` | 確認 thrustBase 準確；ki 降低 |
| 緩慢下沉 | `thrustBase` 或 `pidVZ.ki` | 提高 thrustBase；ki 升高 |
| 起飛超衝 | `pidVZ.ki` | 降低 |
| 懸停漂移 | `pidAngle.ki` 或 `trim` | ki 升高；trim 補偏 |
| 高頻振盪（嗡嗡） | `pidAngle.kp` | 降低 |
| 低頻搖擺 | `pidAngle.ki` | 降低 |
| 旋轉偏轉 | `pidAngle.yaw.kp` | 提高 |
| 姿態讀值高頻跳動 | `Mahony Kp` | 降低（0.4 → 0.3） |
| 長時懸停姿態累積偏移 | `Mahony Ki` | 升高（0.001 → 0.002） |
