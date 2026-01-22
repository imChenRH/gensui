# UWB单基站跟随套件 - 二维平面移动路径可视化程序

## 功能说明

本程序从UWB基站接收串口数据，实时可视化显示信标的**移动路径**。

使用三个二维平面视图：
- **俯视图 (X-Y平面)**：左右和前后方向
- **前视图 (X-Z平面)**：左右和上下方向
- **侧视图 (Y-Z平面)**：前后和上下方向

### 主要功能

- ✅ 三个二维平面实时显示X、Y、Z坐标
- ✅ 路径渐隐效果：旧路径5秒后自动消失
- ✅ 实时数据显示：距离、方位角、仰角、XYZ坐标
- ✅ 自动调整显示范围
- ✅ 支持模拟模式（无需硬件测试）

## 依赖安装

```bash
pip install pyserial matplotlib numpy
```

## 使用方法

### 1. 查看帮助

```bash
python uwb_path_visualizer.py --help
```

### 2. 列出可用串口

```bash
python uwb_path_visualizer.py --list
```

### 3. 连接真实硬件

```bash
# Windows
python uwb_path_visualizer.py COM3

# Linux/Mac
python uwb_path_visualizer.py /dev/ttyUSB0
```

### 4. 模拟模式（无需硬件）

```bash
python uwb_path_visualizer.py --simulate
```

## 界面布局

```
┌─────────────────────┬─────────────────────┐
│   俯视图 (X-Y)      │   前视图 (X-Z)      │
│   左右 vs 前后      │   左右 vs 上下      │
├─────────────────────┼─────────────────────┤
│   侧视图 (Y-Z)      │   实时数据信息      │
│   前后 vs 上下      │                     │
└─────────────────────┴─────────────────────┘
```

## 坐标系说明

```
        Y+ (前方)
        ↑
        |    Z+ (向上)
        |   /
        |  /
 X- ←---+---→ X+ (右侧)
        |
        ↓
        Y- (后方)
```

- 🔴 **红色方块**: 基站位置（坐标原点）
- 🟢 **绿色圆点**: 信标当前位置
- 🔵 **蓝色线条**: 移动轨迹

## 信息面板显示

- 数据点数
- 距离（cm）
- 方位角（度）
- 仰角（度）
- X坐标（cm）
- Y坐标（cm）
- Z坐标（cm）
- 路径点数量
- 路径残留时间

## 配置修改

在代码顶部可以修改以下参数：

```python
MAX_PATH_POINTS = 100       # 最大保留路径点数
UPDATE_INTERVAL = 50        # 图形更新间隔（毫秒）
DISPLAY_RANGE = 200         # 默认显示范围（厘米）
PATH_FADE_TIME = 5.0        # 路径渐隐时间（秒）
```

## 数据协议

程序遵循以下协议格式（共37字节）：

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| MessageHeader | 0-3 | 4 | 帧头 0xFFFFFFFF |
| PacketLength | 4-5 | 2 | 消息体长度 |
| SequenceID | 6-7 | 2 | 消息流水号 |
| RequestCommand | 8-9 | 2 | 命令码 0x2001 |
| VersionID | 10-11 | 2 | 协议版本 |
| AnchorID | 12-15 | 4 | 基站ID |
| TagID | 16-19 | 4 | 标签ID |
| Distance | 20-23 | 4 | 距离（cm） |
| Azimuth | 24-25 | 2 | 方位角（度） |
| Elevation | 26-27 | 2 | 仰角（度） |

### 坐标转换公式

```
水平距离 = Distance × cos(Elevation)
X = 水平距离 × sin(Azimuth)
Y = 水平距离 × cos(Azimuth)
Z = Distance × sin(Elevation)
```

## 常见问题

### Q: 窗口打开但没有数据显示
A: 请检查：
1. 串口号是否正确
2. 基站是否上电
3. 信标是否在范围内
4. TX/RX是否接反

### Q: 坐标数值异常偏大
A: 请确保使用最新版本的代码，已修正协议解析的字节偏移问题。
