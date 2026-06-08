# PYDRONE pre-flight hardware verification script.
#
# Runs on the current (drone-module-less) official MicroPython firmware to
# verify board wiring, sensors and I2C timing BEFORE the py-drone C modules
# are ported. Does NOT touch the motors.
#
# Usage:
#   mpremote run preflight_check.py
#
# Expected devices on the sensor I2C bus (SCL=15, SDA=16, from mpconfigboard.h):
#   0x68  MPU6050 IMU        (WHO_AM_I reg 0x75 -> 0x68)
#   0x76  SPL06 barometer    (ID reg 0x0D -> 0x10)
#   0x0D  QMC5883L magnetometer (driver is named hmc5883l but chip is QMC)

import sys
import time
import gc
from machine import I2C, Pin

# Pin definitions from boards/PYDRONE/mpconfigboard.h
SENSOR_SCL = 15
SENSOR_SDA = 16
DECK_SCL = 1
DECK_SDA = 6

MPU6050_ADDR = 0x68
SPL06_ADDR = 0x76
QMC5883L_ADDR = 0x0D

results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print("[{}] {}{}".format("PASS" if ok else "FAIL", name, ": " + detail if detail else ""))


print("=" * 60)
print("PYDRONE pre-flight check")
print("=" * 60)

# --- 1. Firmware info -------------------------------------------------------
print("\n-- firmware --")
print(sys.implementation)
print(sys.platform)

# --- 2. SPIRAM --------------------------------------------------------------
print("\n-- memory --")
gc.collect()
free = gc.mem_free()
print("gc.mem_free():", free)
# 8MB octal SPIRAM board: heap should be several MB. Without SPIRAM it is ~100KB.
report("SPIRAM heap", free > 2 * 1024 * 1024, "{:.1f} MB free".format(free / 1024 / 1024))

# --- 3. Sensor I2C bus scan -------------------------------------------------
print("\n-- sensor I2C bus (SCL={}, SDA={}) --".format(SENSOR_SCL, SENSOR_SDA))
i2c = I2C(0, scl=Pin(SENSOR_SCL), sda=Pin(SENSOR_SDA), freq=400000)
found = i2c.scan()
print("scan:", [hex(a) for a in found])
report("MPU6050 present (0x68)", MPU6050_ADDR in found)
report("SPL06 present (0x76)", SPL06_ADDR in found)
report("QMC5883L present (0x0D)", QMC5883L_ADDR in found)

# --- 4. Chip ID checks ------------------------------------------------------
print("\n-- chip IDs --")
if MPU6050_ADDR in found:
    who = i2c.readfrom_mem(MPU6050_ADDR, 0x75, 1)[0]
    report("MPU6050 WHO_AM_I", who == 0x68, hex(who))
if SPL06_ADDR in found:
    chip = i2c.readfrom_mem(SPL06_ADDR, 0x0D, 1)[0]
    report("SPL06 chip ID", chip == 0x10, hex(chip))

# --- 5. IMU gravity sanity check -------------------------------------------
# Wake the MPU6050 and verify the accelerometer sees ~1g. Board should be
# stationary and roughly level when running this.
print("\n-- IMU sanity (board must be stationary) --")
if MPU6050_ADDR in found:
    i2c.writeto_mem(MPU6050_ADDR, 0x6B, b"\x01")  # PWR_MGMT_1: wake, PLL x-gyro
    time.sleep_ms(100)
    raw = i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 6)
    ax = (raw[0] << 8 | raw[1])
    ay = (raw[2] << 8 | raw[3])
    az = (raw[4] << 8 | raw[5])
    # sign-extend
    ax = ax - 65536 if ax > 32767 else ax
    ay = ay - 65536 if ay > 32767 else ay
    az = az - 65536 if az > 32767 else az
    # default full scale +/-2g -> 16384 LSB/g
    g = (ax * ax + ay * ay + az * az) ** 0.5 / 16384
    print("acc raw: x={} y={} z={}  |g|={:.3f}".format(ax, ay, az, g))
    report("accelerometer ~1g", 0.8 < g < 1.2, "{:.3f} g".format(g))

# --- 6. I2C read timing benchmark -------------------------------------------
# This answers the sensor-read-duration concern: the flight controller reads
# a 28-byte burst (IMU 14 + mag 7 + baro 7) on every 1 kHz IMU interrupt.
# Theoretical time at 400 kHz is ~0.72 ms for 28 bytes, ~0.4 ms for 14 bytes.
# If the 28-byte read approaches or exceeds 1000 us, samples WILL be dropped.
print("\n-- I2C timing benchmark (400kHz, 500 iterations) --")
if MPU6050_ADDR in found:
    for nbytes in (14, 28):
        buf = bytearray(nbytes)
        tmin, tmax, tsum = 999999, 0, 0
        n = 500
        for _ in range(n):
            t0 = time.ticks_us()
            i2c.readfrom_mem_into(MPU6050_ADDR, 0x3B, buf)
            dt = time.ticks_diff(time.ticks_us(), t0)
            tsum += dt
            if dt < tmin:
                tmin = dt
            if dt > tmax:
                tmax = dt
        avg = tsum / n
        print("{:2d} bytes: min={} avg={:.0f} max={} us".format(nbytes, tmin, avg, tmax))
        if nbytes == 28:
            # Budget: 1 ms sample period. Flag if average leaves <20% headroom.
            report("28B read fits 1kHz budget", avg < 800, "avg {:.0f} us of 1000 us".format(avg))

# --- 7. Deck I2C bus --------------------------------------------------------
print("\n-- deck I2C bus (SCL={}, SDA={}) --".format(DECK_SCL, DECK_SDA))
try:
    deck = I2C(1, scl=Pin(DECK_SCL), sda=Pin(DECK_SDA), freq=100000)
    print("scan:", [hex(a) for a in deck.scan()])
except Exception as e:
    print("deck bus init failed:", e)

# --- Summary -----------------------------------------------------------------
print("\n" + "=" * 60)
failed = [name for name, ok in results if not ok]
print("{}/{} checks passed".format(len(results) - len(failed), len(results)))
if failed:
    print("FAILED:", ", ".join(failed))
else:
    print("All checks passed - hardware layer OK for porting work.")
