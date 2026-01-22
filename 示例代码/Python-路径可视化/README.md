# UWB单基站跟随套件 - 移动路径可视化程序

## 功能说明

本程序从UWB基站接收串口数据，实时可视化显示信标的移动路径。

![可视化示例](visualization_example.png)

### 主要功能

- ✅ 实时接收UWB定位数据
- ✅ 实时绘制移动路径
- ✅ 显示当前位置和起始位置
- ✅ 自动调整显示范围
- ✅ 显示距离、角度等实时数据
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

模拟模式会生成一个螺旋线轨迹，用于测试可视化功能。

## 界面说明

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
```

- 🔴 **红色方块**: 基站位置（坐标原点）
- 🟢 **绿色圆点**: 信标当前位置
- 🔵 **蓝色线条**: 移动轨迹
- 🔺 **绿色三角**: 起始位置

## 信息面板

左上角信息面板显示：
- 接收到的数据点数
- 当前距离（cm）
- 当前方位角（度）
- XY坐标（cm）
- 路径点数量

## 配置修改

在代码顶部可以修改以下参数：

```python
MAX_PATH_POINTS = 500       # 最大保留路径点数
UPDATE_INTERVAL = 50        # 图形更新间隔（毫秒）
GRID_SIZE = 50              # 网格大小（厘米）
DISPLAY_RANGE = 500         # 默认显示范围（厘米）
```

## 常见问题

### Q: 窗口打开但没有数据显示
A: 请检查：
1. 串口号是否正确
2. 基站是否上电
3. 信标是否在范围内
4. TX/RX是否接反

### Q: 中文显示乱码
A: 程序会自动尝试使用SimHei字体。如果仍有问题，请安装中文字体或修改代码中的字体设置。

### Q: 路径显示不流畅
A: 可以尝试：
1. 减小 `UPDATE_INTERVAL`（如改为30）
2. 减小 `MAX_PATH_POINTS`（如改为200）

## 数据协议

程序遵循以下协议格式（共37字节）：

| 字段 | 长度 | 说明 |
|------|------|------|
| MessageHeader | 4 | 帧头 0xFFFFFFFF |
| PacketLength | 2 | 消息体长度 |
| SequenceID | 2 | 消息流水号 |
| RequestCommand | 2 | 命令码 0x2001 |
| VersionID | 2 | 协议版本 |
| AnchorID | 4 | 基站ID |
| TagID | 4 | 标签ID |
| Distance | 4 | 距离（cm） |
| Azimuth | 2 | 方位角（度） |
| Elevation | 2 | 仰角（度） |
| TagStatus | 2 | 标签状态 |
| BatchSn | 2 | 测距序号 |
| Reserve | 4 | 预留 |
| XorByte | 1 | 异或校验 |
