# UWB单基站跟随套件 - C++串口接收程序

## 功能说明

本程序用于从UWB基站接收串口数据，解析定位信息，并将距离和角度转换为XYZ坐标，适用于机器狗跟随等应用场景。

**注意：上传数据均自带卡尔曼滤波。**

## 编译方法

### 方法一：使用MinGW (推荐)

```bash
g++ -o uwb_receiver.exe uwb_serial_receiver.cpp -std=c++11
```

### 方法二：使用Visual Studio

1. 创建新的"控制台应用程序"项目
2. 将 `uwb_serial_receiver.cpp` 添加到项目中
3. 编译运行

## 使用方法

### 1. 硬件连接

1. 使用USB转TTL模块连接基站和电脑
2. 连接方式：
   - 基站TX → USB转TTL的RX
   - 基站RX → USB转TTL的TX
   - GND → GND
3. 安装驱动：运行 `CP2102 USB转TTL驱动.exe`

### 2. 查看串口号

1. 打开"设备管理器"
2. 展开"端口(COM和LPT)"
3. 找到对应的COM端口号

### 3. 运行程序

```bash
# 使用默认串口COM3
uwb_receiver.exe

# 指定串口号
uwb_receiver.exe COM5
```

## 输出示例

```
========================================
【UWB定位数据】
----------------------------------------
  基站ID:    0xAAA2
  信标ID:    0xAAA1
  距离:      25 cm
  方位角:    18 度
  俯仰角:    -10 度
----------------------------------------
【转换后坐标】（单位：厘米）
  X (左右):  7.5 cm
  Y (前后):  23.2 cm
  Z (上下):  -4.3 cm
========================================
```

## 坐标系说明

```
        Y+ (前方)
        ↑
        |
        |
 X- ←---+---→ X+ (右侧)
        |
        |
        ↓
        Y- (后方)

Z+: 向上
Z-: 向下
```

## 数据协议

通信参数：
- 波特率: 115200
- 数据位: 8
- 停止位: 1
- 校验: 无
- 流程控制: 无

数据帧格式（共37字节）：

| 字段 | 长度 | 类型 | 说明 |
|------|------|------|------|
| MessageHeader | 4 | unsigned int | 帧头 0xFFFFFFFF |
| PacketLength | 2 | unsigned int | 消息体长度 |
| SequenceID | 2 | unsigned int | 消息流水号 |
| RequestCommand | 2 | unsigned int | 命令码 0x2001 |
| VersionID | 2 | unsigned int | 协议版本 0x0100 |
| AnchorID | 4 | unsigned int | 基站ID |
| TagID | 4 | unsigned int | 标签ID |
| Distance | 4 | unsigned int | 距离，单位：cm |
| Azimuth | 2 | signed int | 方位角，单位：度 |
| Elevation | 2 | signed int | 仰角，单位：度 |
| TagStatus | 2 | byte | 标签状态 |
| BatchSn | 2 | byte | 测距序号 |
| Reserve | 4 | byte | 预留 |
| XorByte | 1 | byte | 异或校验 |

协议示例：
- 距离25cm，角度18度：`FF FF FF FF 00 25 00 0B 20 01 01 00 00 00 AA A2 00 00 AA A1 00 00 00 19 00 12 FF CA 12 34 00 0B 00 00 00 00 1E`
- 距离98cm，角度-39度：`FF FF FF FF 00 25 00 27 20 01 01 00 00 00 AA A2 00 00 AA A1 00 00 00 62 FF D9 00 0D 12 34 00 27 00 00 00 00 69`

## 常见问题

### Q: 无法打开串口
A: 请检查：
1. 串口是否被其他程序占用
2. USB转TTL驱动是否安装
3. 串口号是否正确

### Q: 收不到数据
A: 请检查：
1. TX/RX是否接反
2. 基站是否上电
3. 信标是否在范围内
