# Betaflight 自定义传感器遥测协议 V1

## 1. 功能概览

飞控通过指定 UART 的 TX 引脚，以固定二进制帧发送以下数据：

- GPS 地速和海拔高度
- 气压计相对高度，上电校准位置为零点
- MS4525 空速计指示空速
- IMU 三轴陀螺仪角速度
- IMU 三轴加速度，包含重力
- 传感器有效状态和飞控解锁状态

串口默认参数：

| 参数 | 值 |
|---|---:|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 流控 | 无 |
| 发送方向 | 飞控 TX 单向输出 |
| 发送频率 | 约 50 Hz |

## 2. 飞控设置

刷入对应固件后进入 Betaflight CLI。先使用 `get telemetry_` 和 `get *_uart` 检查 UART 是否已被接收机、GPS、MSP、OSD、ESC 遥测等功能占用。

以下示例使用 UART4：

```text
set telemetry_1_protocol = CUSTOM
set telemetry_1_uart = UART4
set telemetry_1_baud = 115200
set telemetry_inverted = OFF

set pitot_bustype = I2C
set pitot_i2c_device = 1
set pitot_i2c_address = 0
set pitot_hardware = MS4525

save
```

`pitot_i2c_address = 0` 表示使用驱动默认地址 `0x28`。如果 `telemetry_1` 已占用，可使用 `telemetry_2` 或 `telemetry_3`。

保存并重启时保持空速管静止、无气流。空速计会在未解锁状态采集约 1 秒零点；完成前空速有效位为 0。校准完成后可以用接收帧的状态位确认，不要仅根据空速数值是否为零判断。

## 3. 二进制帧

每帧固定 40 字节。所有多字节整数均为小端序，负数采用二进制补码。

| 偏移 | 长度 | 类型 | 字段 | 单位/说明 |
|---:|---:|---|---|---|
| 0 | 2 | `uint8[2]` | 帧头 | 固定 `0xAA 0x55` |
| 2 | 1 | `uint8` | 协议版本 | 固定 `1` |
| 3 | 1 | `uint8` | 总帧长度 | 固定 `40` (`0x28`) |
| 4 | 2 | `uint16` | 序号 | 每次生成帧加 1，溢出回零 |
| 6 | 4 | `uint32` | 启动时间 | ms，约 49.7 天回绕 |
| 10 | 2 | `uint16` | 有效/状态位 | 见下一节 |
| 12 | 2 | `uint16` | GPS 地速 | cm/s |
| 14 | 4 | `int32` | GPS 海拔 | cm，通常为 MSL 高度 |
| 18 | 4 | `int32` | 气压计高度 | cm，上电位置为零，可为负数 |
| 22 | 4 | `int32` | 指示空速 | cm/s，可表示反向压差 |
| 26 | 2 | `int16` | Gyro X/Roll | 0.1 deg/s |
| 28 | 2 | `int16` | Gyro Y/Pitch | 0.1 deg/s |
| 30 | 2 | `int16` | Gyro Z/Yaw | 0.1 deg/s |
| 32 | 2 | `int16` | Acc X | mg，包含重力 |
| 34 | 2 | `int16` | Acc Y | mg，包含重力 |
| 36 | 2 | `int16` | Acc Z | mg，包含重力 |
| 38 | 2 | `uint16` | CRC | CRC-16/CCITT，小端序 |

换算方法：

```text
gyro_deg_s = gyro_dps10 / 10.0
acc_g      = acc_mg / 1000.0
speed_m_s  = speed_cm_s / 100.0
height_m   = height_cm / 100.0
```

IMU 数据使用经过板方向修正的飞控机体系 X/Y/Z。陀螺仪数据已经校准和滤波。加速度计包含重力，静止水平放置时 Z 轴绝对值通常接近 `1000 mg`。

## 4. 有效和状态位

偏移 10 的 `uint16` 按位定义如下：

| 位 | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x0001` | GPS 传感器已检测 |
| 1 | `0x0002` | GPS 已获得有效定位 |
| 2 | `0x0004` | 气压计存在且校准完成 |
| 3 | `0x0008` | 空速计存在且校准完成 |
| 4 | `0x0010` | 陀螺仪存在且校准完成 |
| 5 | `0x0020` | 加速度计已有有效采样 |
| 6 | `0x0040` | 飞控已解锁 |
| 7..15 | - | 保留，接收端应忽略 |

字段为零不等于传感器无效。例如静止时速度本来就是零。地面站必须先检查对应有效位。

## 5. CRC 校验

CRC 参数：

```text
算法：CRC-16/CCITT-FALSE
多项式：0x1021
初始值：0xFFFF
输入反射：否
输出反射：否
最终异或：0x0000
校验范围：帧偏移 2 至 37，共 36 字节
线上 CRC 字节顺序：低字节在前，高字节在后
```

C 语言参考实现：

```c
uint16_t customCrc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--) {
        crc ^= (uint16_t)*data++ << 8;
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }

    return crc;
}

const uint16_t calculated = customCrc16(&frame[2], 36);
const uint16_t received = frame[38] | ((uint16_t)frame[39] << 8);
```
