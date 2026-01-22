/**
 * ============================================================================
 * @file    uwb_serial_receiver.cpp
 * @brief   UWB单基站跟随套件 - Windows串口数据接收与坐标转换程序
 * @author  Copilot
 * @date    2026-01-21
 * 
 * @description
 * 本程序用于从UWB基站接收串口数据，解析定位信息，并将距离和角度
 * 转换为二维XY坐标，适用于机器狗跟随等应用场景。
 * 
 * @hardware
 * - UWB基站通过USB转TTL模块连接到电脑
 * - 串口参数：115200波特率，8数据位，1停止位，无校验
 * 
 * @compile
 * 使用Visual Studio或MinGW编译：
 * g++ -o uwb_receiver uwb_serial_receiver.cpp -std=c++11
 * ============================================================================
 */

#include <windows.h>    // Windows API，用于串口操作
#include <iostream>     // 标准输入输出
#include <cmath>        // 数学函数（sin, cos等）
#include <cstdint>      // 标准整数类型
#include <cstring>      // 内存操作函数

// ============================================================================
// 宏定义
// ============================================================================

#define FRAME_HEADER        0xFFFFFFFF  // 数据帧头标识
#define CMD_POSITION_DATA   0x2001      // 位置数据命令字
#define FRAME_LENGTH        37          // 完整数据帧长度（字节）- 根据协议：4帧头+2长度+2流号+2命令+2版本+4基站ID+4标签ID+4距离+2方位角+2仰角+2状态+2序号+4预留+1校验=37
#define BUFFER_SIZE         256         // 接收缓冲区大小
#define PI                  3.14159265358979323846  // 圆周率

// ============================================================================
// 数据结构定义
// ============================================================================

/**
 * @brief UWB数据帧结构体
 * @note  数据采用大端序(Big-Endian)传输，需要进行字节序转换
 * 
 * 协议格式（共37字节）：
 * - MessageHeader:   4字节  帧头 0xFFFFFFFF
 * - PacketLength:    2字节  消息体长度
 * - SequenceID:      2字节  消息流水号
 * - RequestCommand:  2字节  命令码 0x2001
 * - VersionID:       2字节  协议版本 0x0100
 * - AnchorID:        4字节  基站ID
 * - TagID:           4字节  标签ID
 * - Distance:        4字节  距离，单位：cm（厘米）
 * - Azimuth:         2字节  方位角，单位：度
 * - Elevation:       2字节  仰角，单位：度
 * - TagStatus:       2字节  标签状态
 * - BatchSn:         2字节  测距序号
 * - Reserve:         4字节  预留
 * - XorByte:         1字节  异或校验
 */
#pragma pack(push, 1)  // 取消字节对齐，确保结构体紧凑排列
typedef struct {
    uint32_t frameHeader;      // 帧头: 0xFFFFFFFF (4字节)
    uint16_t packetLength;     // 数据包长度 (2字节)
    uint16_t sequenceID;       // 序列号 (2字节)
    uint16_t requestCommand;   // 命令字: 0x2001表示位置数据 (2字节)
    uint16_t versionID;        // 版本号: 0x0100 (2字节)
    uint32_t anchorID;         // 基站ID (4字节)
    uint32_t tagID;            // 信标ID (4字节)
    uint32_t distance;         // 距离，单位：cm（厘米）(4字节)
    int16_t  azimuth;          // 方位角，单位：度 (2字节)
    int16_t  elevation;        // 俯仰角（仰角），单位：度 (2字节)
    uint16_t tagStatus;        // 信标状态 (2字节)
    uint16_t batchSn;          // 测距序号 (2字节)
    uint32_t reserve;          // 预留字段 (4字节)
    uint8_t  xorCheck;         // 异或校验 (1字节)
} UWBDataFrame;
#pragma pack(pop)

/**
 * @brief 坐标结构体
 * @note  用于存储转换后的二维坐标，单位统一为厘米(cm)
 */
typedef struct {
    double x;       // X坐标（单位：厘米）
    double y;       // Y坐标（单位：厘米）
    double z;       // Z坐标（单位：厘米，可选）
} Coordinate3D;

/**
 * @brief 定位信息结构体
 * @note  解析后的完整定位信息
 */
typedef struct {
    uint32_t    distance_cm;    // 距离（厘米）
    double      azimuth_deg;    // 方位角（度）
    double      elevation_deg;  // 俯仰角（度）
    Coordinate3D coord;         // 转换后的坐标（厘米）
    uint32_t    tagID;          // 信标ID
    uint32_t    anchorID;       // 基站ID
    bool        valid;          // 数据是否有效
} PositionInfo;

// ============================================================================
// 全局变量
// ============================================================================

HANDLE hSerial = INVALID_HANDLE_VALUE;  // 串口句柄
uint8_t rxBuffer[BUFFER_SIZE];          // 接收缓冲区
int rxBufferIndex = 0;                  // 缓冲区当前索引

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 16位大端序转小端序
 * @param value 大端序数据
 * @return 小端序数据
 * @note  UWB基站发送的数据是大端序，Windows系统是小端序，需要转换
 */
uint16_t swapEndian16(uint16_t value) {
    return ((value & 0x00FF) << 8) | 
           ((value & 0xFF00) >> 8);
}

/**
 * @brief 32位大端序转小端序
 * @param value 大端序数据
 * @return 小端序数据
 */
uint32_t swapEndian32(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8)  |
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0xFF000000) >> 24);
}

/**
 * @brief 计算异或校验值
 * @param data 数据缓冲区
 * @param length 数据长度（不包含校验字节本身）
 * @return 异或校验结果
 * @note  校验方式：该字节前所有字节的异或
 */
uint8_t calculateXorCheck(uint8_t* data, int length) {
    uint8_t xorValue = 0;
    for (int i = 0; i < length; i++) {
        xorValue ^= data[i];
    }
    return xorValue;
}
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0xFF000000) >> 24);
}

/**
 * @brief 角度转弧度
 * @param degrees 角度值
 * @return 弧度值
 */
double degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

/**
 * @brief 将球坐标（距离、方位角、俯仰角）转换为笛卡尔坐标（X, Y, Z）
 * @param distance_cm  距离（厘米）
 * @param azimuth_deg  方位角（度），正前方为0度，顺时针为正
 * @param elevation_deg 俯仰角（度），水平为0度，向上为正
 * @return 三维坐标（厘米）
 * 
 * @note 坐标系定义：
 *       - Y轴：正前方
 *       - X轴：右侧为正
 *       - Z轴：向上为正
 * 
 * @formula
 *       x = distance * cos(elevation) * sin(azimuth)
 *       y = distance * cos(elevation) * cos(azimuth)
 *       z = distance * sin(elevation)
 */
Coordinate3D sphericalToCartesian(double distance_cm, double azimuth_deg, double elevation_deg) {
    Coordinate3D coord;
    
    // 将角度转换为弧度
    double azimuth_rad = degreesToRadians(azimuth_deg);
    double elevation_rad = degreesToRadians(elevation_deg);
    
    // 计算三维坐标
    // cos(elevation) 是水平投影系数
    double horizontalDistance = distance_cm * cos(elevation_rad);
    
    // X坐标：左右方向，右侧为正
    coord.x = horizontalDistance * sin(azimuth_rad);
    
    // Y坐标：前后方向，前方为正
    coord.y = horizontalDistance * cos(azimuth_rad);
    
    // Z坐标：上下方向，向上为正
    coord.z = distance_cm * sin(elevation_rad);
    
    return coord;
}

// ============================================================================
// 串口操作函数
// ============================================================================

/**
 * @brief 初始化并打开串口
 * @param portName 串口名称，如 "COM3"
 * @param baudRate 波特率，默认115200
 * @return 成功返回true，失败返回false
 */
bool openSerialPort(const char* portName, DWORD baudRate = 115200) {
    // 构造完整的串口路径（支持COM10及以上）
    char fullPortName[32];
    sprintf(fullPortName, "\\\\.\\%s", portName);
    
    // 打开串口
    hSerial = CreateFileA(
        fullPortName,                       // 串口名称
        GENERIC_READ | GENERIC_WRITE,       // 读写权限
        0,                                  // 不共享
        NULL,                               // 默认安全属性
        OPEN_EXISTING,                      // 打开已存在的设备
        0,                                  // 同步模式
        NULL                                // 无模板
    );
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "[错误] 无法打开串口 " << portName << std::endl;
        std::cerr << "       请检查：" << std::endl;
        std::cerr << "       1. 串口是否已被其他程序占用" << std::endl;
        std::cerr << "       2. USB转TTL模块是否已正确连接" << std::endl;
        std::cerr << "       3. 驱动程序是否已正确安装" << std::endl;
        return false;
    }
    
    // 配置串口参数
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    // 获取当前串口配置
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "[错误] 无法获取串口配置" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    // 设置串口参数
    dcbSerialParams.BaudRate = baudRate;        // 波特率：115200
    dcbSerialParams.ByteSize = 8;               // 数据位：8位
    dcbSerialParams.StopBits = ONESTOPBIT;      // 停止位：1位
    dcbSerialParams.Parity   = NOPARITY;        // 校验：无
    
    // 应用串口配置
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "[错误] 无法设置串口参数" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    // 设置超时参数
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;      // 字符间最大间隔（毫秒）
    timeouts.ReadTotalTimeoutConstant    = 50;      // 读取总超时常量
    timeouts.ReadTotalTimeoutMultiplier  = 10;      // 读取总超时乘数
    timeouts.WriteTotalTimeoutConstant   = 50;      // 写入总超时常量
    timeouts.WriteTotalTimeoutMultiplier = 10;      // 写入总超时乘数
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "[错误] 无法设置串口超时" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    // 清空串口缓冲区
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    
    std::cout << "[成功] 串口 " << portName << " 已打开" << std::endl;
    std::cout << "       波特率: " << baudRate << std::endl;
    std::cout << "       配置: 8N1 (8数据位, 无校验, 1停止位)" << std::endl;
    
    return true;
}

/**
 * @brief 关闭串口
 */
void closeSerialPort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        std::cout << "[信息] 串口已关闭" << std::endl;
    }
}

/**
 * @brief 从串口读取数据
 * @param buffer 数据缓冲区
 * @param bufferSize 缓冲区大小
 * @return 实际读取的字节数，失败返回-1
 */
int readSerialData(uint8_t* buffer, int bufferSize) {
    DWORD bytesRead = 0;
    
    if (!ReadFile(hSerial, buffer, bufferSize, &bytesRead, NULL)) {
        return -1;
    }
    
    return (int)bytesRead;
}

// ============================================================================
// 数据解析函数
// ============================================================================

/**
 * @brief 在缓冲区中查找帧头
 * @param buffer 数据缓冲区
 * @param length 缓冲区有效数据长度
 * @return 帧头位置索引，未找到返回-1
 */
int findFrameHeader(uint8_t* buffer, int length) {
    // 帧头是4个连续的0xFF字节
    for (int i = 0; i <= length - 4; i++) {
        if (buffer[i]   == 0xFF && 
            buffer[i+1] == 0xFF && 
            buffer[i+2] == 0xFF && 
            buffer[i+3] == 0xFF) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 解析UWB数据帧
 * @param frameData 完整的数据帧（37字节）
 * @param posInfo   输出的位置信息
 * @return 解析成功返回true，失败返回false
 * 
 * @note 协议示例1：距离25cm，角度18度
 *       FF FF FF FF 00 25 00 0B 20 01 01 00 00 00 AA A2 00 00 AA A1 00 00 00 19 00 12 FF CA 12 34 00 0B 00 00 00 00 1E
 *       距离=0x19=25cm，方位角=0x12=18度
 * 
 * @note 协议示例2：距离98cm，角度-39度
 *       FF FF FF FF 00 25 00 27 20 01 01 00 00 00 AA A2 00 00 AA A1 00 00 00 62 FF D9 00 0D 12 34 00 27 00 00 00 00 69
 *       距离=0x62=98cm，方位角=0xFFD9=-39度（有符号）
 */
bool parseUWBFrame(uint8_t* frameData, PositionInfo* posInfo) {
    // 将字节数据映射到结构体
    UWBDataFrame* frame = (UWBDataFrame*)frameData;
    
    // 验证异或校验（校验字节前所有字节的异或）
    uint8_t calculatedXor = calculateXorCheck(frameData, FRAME_LENGTH - 1);
    if (calculatedXor != frame->xorCheck) {
        // 校验失败，数据可能损坏
        std::cerr << "[警告] 数据校验失败，丢弃该帧" << std::endl;
        posInfo->valid = false;
        return false;
    }
    
    // 转换字节序并验证命令字
    uint16_t command = swapEndian16(frame->requestCommand);
    
    // 检查是否是位置数据命令
    if (command != CMD_POSITION_DATA) {
        posInfo->valid = false;
        return false;
    }
    
    // 解析并转换各字段（大端序 -> 小端序）
    // 距离单位：厘米(cm)
    posInfo->distance_cm = swapEndian32(frame->distance);
    posInfo->tagID = swapEndian32(frame->tagID);
    posInfo->anchorID = swapEndian32(frame->anchorID);
    
    // 方位角和俯仰角：单位直接是度，不需要除以100
    // 注意：这是有符号整数，负值表示反方向
    int16_t rawAzimuth = (int16_t)swapEndian16((uint16_t)frame->azimuth);
    int16_t rawElevation = (int16_t)swapEndian16((uint16_t)frame->elevation);
    
    posInfo->azimuth_deg = (double)rawAzimuth;      // 方位角（度）
    posInfo->elevation_deg = (double)rawElevation;  // 俯仰角（度）
    
    // 将球坐标转换为笛卡尔坐标（单位：厘米）
    posInfo->coord = sphericalToCartesian(
        (double)posInfo->distance_cm,
        posInfo->azimuth_deg,
        posInfo->elevation_deg
    );
    
    posInfo->valid = true;
    return true;
}

/**
 * @brief 处理接收缓冲区中的数据
 * @param posInfo 输出的位置信息
 * @return 成功解析到数据返回true
 */
bool processReceivedData(PositionInfo* posInfo) {
    // 查找帧头位置
    int headerIndex = findFrameHeader(rxBuffer, rxBufferIndex);
    
    if (headerIndex < 0) {
        // 未找到帧头，清空无效数据（保留最后3字节，可能是不完整的帧头）
        if (rxBufferIndex > 3) {
            memmove(rxBuffer, rxBuffer + rxBufferIndex - 3, 3);
            rxBufferIndex = 3;
        }
        return false;
    }
    
    // 检查帧头之前是否有无效数据
    if (headerIndex > 0) {
        // 移除帧头之前的无效数据
        memmove(rxBuffer, rxBuffer + headerIndex, rxBufferIndex - headerIndex);
        rxBufferIndex -= headerIndex;
    }
    
    // 检查是否有完整的数据帧
    if (rxBufferIndex < FRAME_LENGTH) {
        // 数据不完整，等待更多数据
        return false;
    }
    
    // 解析数据帧
    bool result = parseUWBFrame(rxBuffer, posInfo);
    
    // 移除已处理的数据帧
    memmove(rxBuffer, rxBuffer + FRAME_LENGTH, rxBufferIndex - FRAME_LENGTH);
    rxBufferIndex -= FRAME_LENGTH;
    
    return result;
}

// ============================================================================
// 显示函数
// ============================================================================

/**
 * @brief 打印位置信息
 * @param posInfo 位置信息结构体
 */
void printPositionInfo(const PositionInfo& posInfo) {
    std::cout << "========================================" << std::endl;
    std::cout << "【UWB定位数据】" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  基站ID:    0x" << std::hex << posInfo.anchorID << std::dec << std::endl;
    std::cout << "  信标ID:    0x" << std::hex << posInfo.tagID << std::dec << std::endl;
    std::cout << "  距离:      " << posInfo.distance_cm << " cm" << std::endl;
    std::cout << "  方位角:    " << posInfo.azimuth_deg << " 度" << std::endl;
    std::cout << "  俯仰角:    " << posInfo.elevation_deg << " 度" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "【转换后坐标】（单位：厘米）" << std::endl;
    std::cout << "  X (左右):  " << posInfo.coord.x << " cm" << std::endl;
    std::cout << "  Y (前后):  " << posInfo.coord.y << " cm" << std::endl;
    std::cout << "  Z (上下):  " << posInfo.coord.z << " cm" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 打印使用说明
 */
void printUsage() {
    std::cout << "============================================" << std::endl;
    std::cout << "  UWB单基站跟随套件 - 串口数据接收程序" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;
    std::cout << "用法: uwb_receiver.exe [串口号]" << std::endl;
    std::cout << "示例: uwb_receiver.exe COM3" << std::endl;
    std::cout << std::endl;
    std::cout << "说明:" << std::endl;
    std::cout << "  - 默认串口: COM3" << std::endl;
    std::cout << "  - 波特率: 115200" << std::endl;
    std::cout << "  - 按 Ctrl+C 退出程序" << std::endl;
    std::cout << std::endl;
}

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char* argv[]) {
    // 设置控制台编码为UTF-8（支持中文显示）
    SetConsoleOutputCP(65001);
    
    // 打印使用说明
    printUsage();
    
    // 获取串口号（默认COM3）
    const char* portName = "COM3";
    if (argc >= 2) {
        portName = argv[1];
    }
    
    std::cout << "[信息] 正在打开串口 " << portName << "..." << std::endl;
    
    // 打开串口
    if (!openSerialPort(portName, 115200)) {
        std::cout << "[错误] 串口打开失败，程序退出" << std::endl;
        return 1;
    }
    
    std::cout << std::endl;
    std::cout << "[信息] 开始接收数据，按 Ctrl+C 退出..." << std::endl;
    std::cout << std::endl;
    
    // 位置信息结构体
    PositionInfo posInfo;
    
    // 临时接收缓冲区
    uint8_t tempBuffer[64];
    
    // 主循环：持续接收和处理数据
    while (true) {
        // 从串口读取数据
        int bytesRead = readSerialData(tempBuffer, sizeof(tempBuffer));
        
        if (bytesRead > 0) {
            // 将读取的数据添加到接收缓冲区
            if (rxBufferIndex + bytesRead < BUFFER_SIZE) {
                memcpy(rxBuffer + rxBufferIndex, tempBuffer, bytesRead);
                rxBufferIndex += bytesRead;
            } else {
                // 缓冲区溢出，清空并重新开始
                std::cerr << "[警告] 接收缓冲区溢出，正在重置..." << std::endl;
                rxBufferIndex = 0;
            }
            
            // 尝试处理接收到的数据
            while (processReceivedData(&posInfo)) {
                if (posInfo.valid) {
                    // 打印位置信息
                    printPositionInfo(posInfo);
                }
            }
        }
        
        // 短暂延时，避免CPU占用过高
        Sleep(10);
    }
    
    // 关闭串口（实际上由于while(true)不会执行到这里）
    closeSerialPort();
    
    return 0;
}
