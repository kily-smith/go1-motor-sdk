# Go1 拆机电机 SDK

独立封装实际测试过的 34 字节命令 / 78 字节反馈、CRC32、4.8 Mbps RS-485 协议。提供 Python、Linux C++、STM32 可移植 C 协议与传输回调。接口借鉴宇树 A1 SDK 的 `MotorCmd`、`MotorData`、`SerialPort.sendRecv` 风格，**不是宇树官方 SDK，也不声称 ABI / 行为完全兼容**。

实测样本是一颗拆机电机，具体型号尚未确定；不能假设所有 Go1/Go2 拆机电机同固件、参数或接口。当前样本最近确认 ID 为 **1**。Python 和 Linux C++ 新封装已各用一帧 mode0 在该电机上查询成功，均收到 ID1、mode0、42°C、错误0 的有效反馈；新封装的运动控制和 STM32 实板尚未验证。


📖 [拆机、CRC 与通信测试博客](blog.md)

## 目录与编译

```bash
cd go1-motor-sdk
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
python3 tests/test_integration.py
```

Python 安装到自己的虚拟环境：`python3 -m pip install ./python`；需要 `pyserial`。如果不安装，可在项目根目录执行：

```bash
PYTHONPATH=python python3 examples/stop.py --port /dev/ttyUSB0 --id 1
./build/go1_stop /dev/ttyUSB0 1
```

以上两条命令会实际访问串口、发送 mode 0 停止帧；请按电机当前 ID、端口和实际连接运行。测试文件和模板不会驱动电机。

## Python：A1 风格

```python
from go1_motor_sdk import MotorCmd, MotorData, SerialPort, BRAKE
cmd = MotorCmd(id=1, mode=BRAKE)
data = MotorData()
with SerialPort('/dev/ttyUSB0') as serial:
    serial.sendRecv(cmd, data)  # 默认 50 ms，失败抛 TimeoutError / I/O 异常
    print(data.motor_id, data.mode, data.q, data.dq, data.temp, data.merror)
```

也可 `data = serial.sendRecv(cmd)` 返回新的 `MotorData`。每次调用前清空输入缓存；请求与响应必须串行，不能从多个线程同时操作一个端口。协议反馈没有事务号，已经在途的迟到帧仍可能被误认为新回复；调用者应设计通信周期、等待和异常处理，不要只凭一帧判定新模式。反馈可能反映上一帧命令。`examples/control_modes.py` 给出了位置、速度、阻尼、力矩、零力矩、混合控制的数据构造示例，不自动发送。

## C++：A1 风格

```cpp
#include "go1_serial.hpp"
go1::SerialPort serial("/dev/ttyUSB0");
go1::MotorCmd cmd{};
go1::MotorData data{};
cmd.id = 1;
cmd.mode = go1::BRAKE;
serial.sendRecv(&cmd, &data, 50); // 毫秒；失败抛异常
```

编译时包含 `include`，链接 `go1_serial` 与 `go1_motor`。Linux 实现使用 `termios2/BOTHER` 设置 4.8 Mbps、8N1、独占打开；USB 转换器与驱动必须支持该速率。所有示例默认无持续控制线程，停止后直接关闭端口。

## STM32

将 `include/go1_motor.h`、`src/go1_motor.c`、`stm32/go1_transport.h`、`stm32/go1_transport.c` 加入工程，启用 C99 和数学库，目标平台需使用 32 位 IEEE-754 `float`。无需操作系统、动态内存和指定 HAL。`stm32/hal_example.c` 是 UART + DE GPIO 的移植模板：在选定 STM32 系列后设置 `GO1_STM32_HAL_HEADER`，按芯片实际的 HAL 类型、错误清除与 UART 外设配置调整并加入工程。`Go1Hal` 的定义在模板内；可将它和函数声明提取到工程自己的头文件，或者直接将模板内容合入板级驱动。初始化 `Go1Hal` 后调用 `go1_hal_transport(&hal)` 构建回调，然后：

```c
Go1MotorCmd cmd = {0};
Go1MotorData data = {0};
cmd.id = 1;
cmd.mode = GO1_MODE_STOP;
int rc = go1_send_recv(&io, &cmd, &data, 50); /* 毫秒，0 成功 */
```

UART 设为 **4,800,000 baud、8N1**，按板卡接好共地、RS-485 A/B 和收发方向 DE；不是所有 STM32/UART 时钟都能准确产生此速率，需要根据型号和时钟核对误差。DE 必须保持发送直到 UART **TC（最后停止位发送完毕）**，再及时切回接收；`tx` 回调约定完成这一过程。`rx_reset` 必须清除旧数据，`rx_byte` 必须遵循超时并让毫秒时钟前进；DMA/中断接收需配套环形缓冲和过载处理。RS-485 回显或 UART 溢出因硬件而异，须在实际板卡确认。模板只适合从站响应不会早于本端进入接收状态的情况；未在指定 STM32 或真实收发器验证。

`go1_send_recv` 返回 `0` 成功、`-1` 参数/编码错误、`-2` I/O错误、`-3` 超时。解析器检查包头、预期 ID、长度和 CRC，支持碎片与杂字节；实时任务应由应用负责周期、超时、反馈检查和停止策略。

## 量纲与模式

`MotorCmd` 字段与反馈均是**转子侧**：`q` 弧度、`dq` 弧度/秒、`tau` 按协议比例表示的转子侧力矩值；`kp`、`kd` 是 A1 风格的**线协议系数**。编码为 `tau×256`、`dq×128`、`q×16384/(2π)`、`kp×2048`、`kd×1024`，转换截断到整数；负增益、溢出、非有限值、未知模式、非零停止帧会拒绝。

SDK 的名义减速比为 **6.33:1**：Python 使用 `queryGearRatio()` 或 `GEAR_RATIO`；C++ 使用 `go1::queryGearRatio()`；STM32 C 使用 `go1_query_gear_ratio()` 或 `GO1_GEAR_RATIO`。它是转子转速与电机减速器输出轴转速的比值，不包含腿部后续传动。依据是这颗样本手动一圈估算约 **6.335:1**，接近 6.33；不同电机、机械传动及高精度用途应另行标定。

SDK 不自动乘除减速比，继续使用转子侧数据：例如输出轴目标速度为 `6.28 rad/s`，则 `cmd.dq = 6.28 * queryGearRatio()`；读取输出轴角速度可用 `data.dq / queryGearRatio()`。角度同理，但绝对位置必须考虑零点和编码器回绕。力矩、增益的物理标定与零售 A1 SDK 的附加系数尚未验证，不能直接把示例参数当真实关节刚度。

- `mode=0`：停止，五个控制字段必须为零。
- `mode=10`：FOC / 混合控制。设定参数可构造位置、速度、阻尼、力矩、零力矩等控制。
- `mode=11` 改号：该样本曾验证拨轮操作；本通用运动 SDK **不发送改号、标定、工厂和刷写指令**。

反馈包含 `motor_id`、`mode`、`temp`、`merror`、`tau`、`dq`、`q`、`LW`、三个原始整数和 `correct`。`LW` 是原始 float，并非经过独立标定的安全测速；长时间转动时样本的原始位置曾出现约 23 位回绕，`q` 不自动展开，应用须结合真实设备测量和采样速度处理回绕。温度与错误码需持续监控；SDK 不会自动限速、限力、监测碰撞或保证断线制动。

## 协议依据

本项目基于实机诊断记录及抓包验证。帧结构参考 gooddawg 的 Go1 组帧实现和宇树 A1/B1 数据格式；CRC 实现依据已验证的测试向量。数据包多字节小端，CRC 按小端 32 位字、最高位先处理；反馈 CRC 覆盖前72字节，CRC在字节74～77，期间两字节未纳入校验。**数据帧有效不保证物理控制安全或结果精确。**
