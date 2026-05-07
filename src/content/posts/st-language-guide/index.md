---
title: ST语言完整入门手册 - 结构化文本编程详解
published: 2024-04-14
description: "全面的ST(Structured Text)语言入门指南，涵盖基础语法、数据类型、控制结构、函数块编程及实际应用案例"
tags: ["PLC", "工业自动化", "ST语言", "编程教程", "IEC 61131-3"]
category: 技术教程
draft: false
---

# ST语言完整入门手册

> **提示**: 本手册内容非常详细，建议使用浏览器的搜索功能(Ctrl+F)快速定位所需内容。

## 目录
- [1. ST语言简介](#1-st语言简介)
- [2. 开发环境准备](#2-开发环境准备)
- [3. 基础语法](#3-基础语法)
- [4. 数据类型详解](#4-数据类型详解)
- [5. 变量声明与作用域](#5-变量声明与作用域)
- [6. 运算符](#6-运算符)
- [7. 控制结构](#7-控制结构)
- [8. 函数与函数块](#8-函数与函数块)
- [9. 程序组织单元](#9-程序组织单元)
- [10. 数组与字符串处理](#10-数组与字符串处理)
- [11. 定时器与计数器](#11-定时器与计数器)
- [12. 自锁与互锁逻辑](#12-自锁与互锁逻辑)
- [13. 实际工程案例](#13-实际工程案例)
- [14. 最佳实践与调试技巧](#14-最佳实践与调试技巧)
- [15. 常见问题解答](#15-常见问题解答)

---

## 1. ST语言简介

### 1.1 什么是ST语言？

ST（Structured Text，结构化文本）是 IEC 61131-3 标准定义的五种 PLC 编程语言之一。它是一种高级文本编程语言，语法类似于 Pascal、C 和 Ada，特别适合处理复杂算法、数据处理和数学运算。

### 1.2 ST语言的特点

**优势：**
- ✅ 适合复杂算法和数学计算
- ✅ 代码紧凑，可读性强
- ✅ 支持结构化编程
- ✅ 便于版本控制和代码复用
- ✅ 适合有编程基础的工程师快速上手

**应用场景：**
- 复杂数学运算和数据处理
- 状态机实现
- 配方管理
- 数据记录和报表生成
- 通信协议实现

### 1.3 IEC 61131-3 标准

IEC 61131-3 是国际电工委员会制定的 PLC 编程标准，定义了：
- **五种编程语言**：LD（梯形图）、FBD（功能块图）、SFC（顺序功能图）、IL（指令表）、ST（结构化文本）
- **通用元素**：变量、数据类型、程序组织单元
- **执行模型**：循环扫描机制

---

## 2. 开发环境准备

### 2.1 主流开发平台

| 平台 | 厂商 | 特点 |
|------|------|------|
| TIA Portal | 西门子 | 集成化程度高，支持 S7-1200/1500 |
| Codesys | 3S-Smart Software | 开放平台，多家厂商支持 |
| GX Works3 | 三菱电机 | 适用于三菱 FX/Q/iQ-R 系列 |
| Studio 5000 | 罗克韦尔 | Allen-Bradley PLC 专用 |
| TwinCAT | 倍福 | 基于 PC 的控制方案 |

### 2.2 第一个ST程序

```pascal
PROGRAM HelloWorld
VAR
    message : STRING := 'Hello, ST Language!';
END_VAR

// 主程序逻辑
message := '欢迎学习ST语言';
END_PROGRAM
```

---

## 3. 基础语法

### 3.1 基本语法规则

```pascal
// 单行注释 (* 多行注释 *)

// 语句以分号结尾
variable := value;

// 赋值运算符
x := 10;
y := x + 5;

// 大小写不敏感（大多数平台）
Variable := 10;  // 等同于 variable := 10;
```

### 3.2 标识符命名规则

```pascal
// 合法标识符
temperatureSensor
Motor_Speed
valvePosition1
_MAX_LIMIT

// 非法标识符
123variable    // 不能以数字开头
motor speed    // 不能包含空格
if             // 不能使用关键字
```

**命名建议：**
- 使用有意义的名称
- 采用驼峰命名法或下划线分隔
- 常量使用大写加下划线
- 避免使用缩写（除非是行业通用）

---

## 4. 数据类型详解

### 4.1 基本数据类型

#### 布尔类型
```pascal
VAR
    isRunning : BOOL := FALSE;
    alarmActive : BOOL := TRUE;
    sensorTriggered : BOOL;
END_VAR

// 布尔运算
isRunning := NOT alarmActive;
result := isRunning AND sensorTriggered;
```

#### 整数类型
```pascal
VAR
    // 有符号整数
    counter : INT;           // 16位: -32768 ~ 32767
    largeValue : DINT;       // 32位: -2147483648 ~ 2147483647
    smallNum : SINT;         // 8位: -128 ~ 127
    hugeNum : LINT;          // 64位
    
    // 无符号整数
    positiveCount : UINT;    // 16位: 0 ~ 65535
    bigPositive : UDINT;     // 32位: 0 ~ 4294967295
    byteValue : BYTE;        // 8位: 0 ~ 255
    wordValue : WORD;        // 16位: 0 ~ 65535
END_VAR

// 数值赋值
counter := 100;
largeValue := -50000;
byteValue := 16#FF;  // 十六进制
```

#### 浮点数类型
```pascal
VAR
    temperature : REAL;      // 32位单精度
    precision : LREAL;       // 64位双精度
    ratio : REAL := 3.14159;
END_VAR

// 科学计数法
voltage := 1.5E2;   // 150.0
current := 2.5E-3;  // 0.0025
```

#### 时间和日期类型
```pascal
VAR
    // 时间类型
    cycleTime : TIME := T#100ms;
    timeout : TIME := T#5s;
    longDuration : TIME := T#1h30m;
    
    // 日期和时间
    currentDate : DATE := D#2024-04-14;
    currentTime : TOD := TOD#14:30:00;
    dateTime : DT := DT#2024-04-14-14:30:00;
END_VAR

// 时间运算
elapsedTime := currentTime - startTime;
IF elapsedTime > T#5s THEN
    // 超时处理
END_IF;
```

#### 字符串类型
```pascal
VAR
    deviceName : STRING := 'Motor_01';
    statusMsg : STRING(50) := '设备运行正常';
    charValue : CHAR := 'A';
END_VAR
```

### 4.2 复合数据类型

#### 数组
```pascal
VAR
    // 一维数组
    temperatures : ARRAY[0..9] OF REAL;
    flags : ARRAY[1..16] OF BOOL;
    
    // 多维数组
    matrix : ARRAY[0..2, 0..2] OF INT;
    
    // 带初始值的数组
    presetValues : ARRAY[0..4] OF INT := [10, 20, 30, 40, 50];
END_VAR

// 数组访问
temperatures[0] := 25.5;
matrix[1, 2] := 100;
```

#### 结构体
```pascal
TYPE MotorData :
STRUCT
    name : STRING(20);
    speed : REAL;
    current : REAL;
    temperature : REAL;
    isRunning : BOOL;
    runningHours : DINT;
END_STRUCT
END_TYPE

VAR
    motor1 : MotorData;
    motorArray : ARRAY[0..3] OF MotorData;
END_VAR

// 结构体访问
motor1.name := '主轴电机';
motor1.speed := 1500.0;
motor1.isRunning := TRUE;
```

#### 枚举类型
```pascal
TYPE MachineState :
(
    IDLE := 0,
    RUNNING := 1,
    PAUSED := 2,
    ERROR := 3,
    MAINTENANCE := 4
);
END_TYPE

VAR
    currentState : MachineState := IDLE;
END_VAR

// 枚举使用
CASE currentState OF
    IDLE:
        // 空闲状态处理
    RUNNING:
        // 运行状态处理
    ERROR:
        // 错误状态处理
END_CASE;
```

### 4.3 类型转换

```pascal
VAR
    intValue : INT := 100;
    realValue : REAL;
    stringValue : STRING;
    boolValue : BOOL;
END_VAR

// 隐式转换（自动）
realValue := INT_TO_REAL(intValue);  // 100.0

// 显式转换函数
intValue := REAL_TO_INT(3.14);       // 3
boolValue := INT_TO_BOOL(1);         // TRUE
stringValue := INT_TO_STRING(123);   // '123'

// 常用转换函数
// TO_BOOL, TO_INT, TO_REAL, TO_STRING
// TRUNC, ROUND, ABS
```

---

## 5. 变量声明与作用域

### 5.1 变量声明区域

```pascal
PROGRAM Example
VAR
    // 局部变量 - 每次调用重新初始化
    localCounter : INT := 0;
END_VAR

VAR_INPUT
    // 输入变量 - 从外部传入
    startSignal : BOOL;
    setpoint : REAL;
END_VAR

VAR_OUTPUT
    // 输出变量 - 传递给外部
    running : BOOL;
    actualValue : REAL;
END_VAR

VAR_IN_OUT
    // 输入输出变量 - 双向传递
    dataBuffer : ARRAY[0..9] OF REAL;
END_VAR

VAR_STAT
    // 静态变量 - 保持上次值
    totalCycles : DINT := 0;
END_VAR

VAR CONSTANT
    // 常量 - 不可修改
    MAX_SPEED : REAL := 3000.0;
    MIN_TEMP : REAL := -40.0;
END_VAR

VAR RETAIN
    // 保持变量 - 断电后保持
    operatingHours : DINT;
END_VAR

END_PROGRAM
```

### 5.2 变量作用域

```pascal
// 全局变量（在 GVL 中声明）
VAR_GLOBAL
    systemReady : BOOL;
    emergencyStop : BOOL;
END_VAR

PROGRAM LocalProgram
VAR
    localVar : INT;  // 仅在此程序中可见
END_VAR

    // 可以访问全局变量
    IF systemReady THEN
        localVar := 100;
    END_IF;
END_PROGRAM
```

---

## 6. 运算符

### 6.1 算术运算符

```pascal
VAR
    a, b, result : REAL;
    intResult : INT;
END_VAR

a := 10.0;
b := 3.0;

result := a + b;   // 加法: 13.0
result := a - b;   // 减法: 7.0
result := a * b;   // 乘法: 30.0
result := a / b;   // 除法: 3.333...
intResult := a MOD b;  // 取模: 1
result := a ** 2;  // 幂运算: 100.0（部分平台支持）
```

### 6.2 比较运算符

```pascal
VAR
    x, y : INT;
    isEqual, isGreater : BOOL;
END_VAR

x := 10;
y := 20;

isEqual := (x = y);      // FALSE
isGreater := (x > y);    // FALSE
isGreater := (x < y);    // TRUE
isEqual := (x <> y);     // TRUE (不等于)
isEqual := (x >= 10);    // TRUE
isEqual := (y <= 20);    // TRUE
```

### 6.3 逻辑运算符

```pascal
VAR
    a, b, c : BOOL;
    result : BOOL;
END_VAR

a := TRUE;
b := FALSE;

result := a AND b;   // FALSE
result := a OR b;    // TRUE
result := NOT a;     // FALSE
result := a XOR b;   // TRUE (异或)

// 组合逻辑
result := (a AND b) OR (NOT c);
```

### 6.4 位运算符

```pascal
VAR
    value : BYTE := 16#0F;  // 0000 1111
    mask : BYTE := 16#F0;   // 1111 0000
    result : BYTE;
END_VAR

result := value AND mask;   // 0000 0000
result := value OR mask;    // 1111 1111
result := value XOR mask;   // 1111 1111
result := NOT value;        // 1111 0000
result := SHL(value, 2);    // 左移2位: 0011 1100
result := SHR(value, 2);    // 右移2位: 0000 0011
```

### 6.5 运算符优先级

从高到低：
1. 括号 `()`
2. 函数调用
3. 幂运算 `**`
4. 取负 `-`、逻辑非 `NOT`
5. 乘除 `*` `/` `MOD`
6. 加减 `+` `-`
7. 比较 `<` `>` `<=` `>=`
8. 等于不等于 `=` `<>`
9. 逻辑与 `AND`
10. 逻辑异或 `XOR`
11. 逻辑或 `OR`

---

## 7. 控制结构

### 7.1 条件语句

#### IF-THEN-ELSE
```pascal
VAR
    temperature : REAL;
    heaterOn : BOOL;
    fanSpeed : INT;
END_VAR

// 简单IF
IF temperature > 100.0 THEN
    heaterOn := FALSE;
END_IF;

// IF-ELSE
IF temperature > 80.0 THEN
    heaterOn := FALSE;
ELSE
    heaterOn := TRUE;
END_IF;

// IF-ELSIF-ELSE
IF temperature < 20.0 THEN
    fanSpeed := 0;
ELSIF temperature < 40.0 THEN
    fanSpeed := 50;
ELSIF temperature < 60.0 THEN
    fanSpeed := 75;
ELSE
    fanSpeed := 100;
END_IF;
```

#### CASE 语句
```pascal
VAR
    errorCode : INT;
    errorMessage : STRING(50);
END_VAR

CASE errorCode OF
    0:
        errorMessage := '无错误';
    1:
        errorMessage := '传感器故障';
    2:
        errorMessage := '电机过载';
    3, 4, 5:  // 多个值
        errorMessage := '通信错误';
    10..20:   // 范围
        errorMessage := '参数超出范围';
ELSE
    errorMessage := '未知错误';
END_CASE;
```

### 7.2 循环语句

#### FOR 循环
```pascal
VAR
    i : INT;
    sum : DINT := 0;
    values : ARRAY[0..9] OF INT;
END_VAR

// 基本FOR循环
FOR i := 0 TO 9 DO
    values[i] := i * 10;
END_FOR;

// 带步长
FOR i := 0 TO 100 BY 10 DO
    sum := sum + i;
END_FOR;

// 递减循环
FOR i := 10 TO 1 BY -1 DO
    // 倒计时处理
END_FOR;
```

#### WHILE 循环
```pascal
VAR
    counter : INT := 0;
    total : DINT := 0;
END_VAR

// WHILE循环
WHILE counter < 100 DO
    total := total + counter;
    counter := counter + 1;
END_WHILE;

// 带退出条件
WHILE NOT stopSignal DO
    // 执行任务
    ProcessData();
    
    // 安全检查
    IF errorDetected THEN
        EXIT;  // 退出循环
    END_IF;
END_WHILE;
```

#### REPEAT-UNTIL 循环
```pascal
VAR
    retryCount : INT := 0;
    communicationOK : BOOL;
END_VAR

// 至少执行一次
REPEAT
    TryCommunication();
    retryCount := retryCount + 1;
UNTIL communicationOK OR retryCount >= 3
END_REPEAT;
```

#### 循环控制
```pascal
FOR i := 0 TO 99 DO
    IF skipCondition THEN
        CONTINUE;  // 跳过本次迭代
    END_IF;
    
    IF exitCondition THEN
        EXIT;  // 退出循环
    END_IF;
    
    // 正常处理
    ProcessItem(i);
END_FOR;
```

---

## 8. 函数与函数块

### 8.1 函数（FUNCTION）

函数有返回值，无内部状态：

```pascal
// 函数定义
FUNCTION Add : REAL
VAR_INPUT
    a : REAL;
    b : REAL;
END_VAR

Add := a + b;  // 返回值
END_FUNCTION

// 使用函数
VAR
    result : REAL;
END_VAR

result := Add(10.5, 20.3);
```

**实用函数示例：**

```pascal
// 限幅函数
FUNCTION LimitValue : REAL
VAR_INPUT
    value : REAL;
    minVal : REAL;
    maxVal : REAL;
END_VAR

IF value < minVal THEN
    LimitValue := minVal;
ELSIF value > maxVal THEN
    LimitValue := maxVal;
ELSE
    LimitValue := value;
END_IF;

END_FUNCTION

// 死区函数
FUNCTION DeadBand : REAL
VAR_INPUT
    input : REAL;
    threshold : REAL;
END_VAR

IF ABS(input) < threshold THEN
    DeadBand := 0.0;
ELSE
    DeadBand := input;
END_IF;

END_FUNCTION
```

### 8.2 函数块（FUNCTION_BLOCK）

函数块有内部状态，可保持数据：

```pascal
// 电机控制函数块
FUNCTION_BLOCK FB_MotorControl
VAR_INPUT
    start : BOOL;
    stop : BOOL;
    speedSetpoint : REAL;
    emergencyStop : BOOL;
END_VAR

VAR_OUTPUT
    running : BOOL;
    actualSpeed : REAL;
    fault : BOOL;
    faultCode : INT;
END_VAR

VAR
    // 内部变量
    currentSpeed : REAL := 0.0;
    acceleration : REAL := 100.0;  // 加速度
    deceleration : REAL := 150.0;  // 减速度
    state : INT := 0;  // 0:停止 1:加速 2:运行 3:减速
END_VAR

// 主逻辑
IF emergencyStop THEN
    // 紧急停止
    currentSpeed := 0.0;
    running := FALSE;
    fault := TRUE;
    faultCode := 1;
    state := 0;
    
ELSIF stop OR fault THEN
    // 正常停止 - 减速
    IF currentSpeed > 0.0 THEN
        currentSpeed := currentSpeed - deceleration * CycleTime;
        IF currentSpeed <= 0.0 THEN
            currentSpeed := 0.0;
            running := FALSE;
            state := 0;
        END_IF;
    END_IF;
    
ELSIF start AND NOT fault THEN
    // 启动 - 加速
    running := TRUE;
    fault := FALSE;
    faultCode := 0;
    
    IF currentSpeed < speedSetpoint THEN
        currentSpeed := currentSpeed + acceleration * CycleTime;
        IF currentSpeed >= speedSetpoint THEN
            currentSpeed := speedSetpoint;
            state := 2;  // 运行状态
        ELSE
            state := 1;  // 加速状态
        END_IF;
    ELSIF currentSpeed > speedSetpoint THEN
        currentSpeed := currentSpeed - deceleration * CycleTime;
        IF currentSpeed <= speedSetpoint THEN
            currentSpeed := speedSetpoint;
            state := 2;
        END_IF;
    END_IF;
END_IF;

actualSpeed := currentSpeed;

END_FUNCTION_BLOCK

// 使用函数块
PROGRAM Main
VAR
    motor1 : FB_MotorControl;
    motor2 : FB_MotorControl;
END_VAR

// 调用
motor1(start := TRUE, 
       stop := FALSE, 
       speedSetpoint := 1500.0,
       emergencyStop := FALSE);

// 读取输出
IF motor1.running THEN
    // 电机1运行中
END_IF;

END_PROGRAM
```

---

## 9. 程序组织单元

### 9.1 程序（PROGRAM）

```pascal
PROGRAM MainControl
VAR
    // 程序变量
    cycleCounter : DINT;
    systemState : INT;
END_VAR

// 主逻辑
cycleCounter := cycleCounter + 1;

CASE systemState OF
    0: // 初始化
        InitializeSystem();
        systemState := 1;
        
    1: // 运行
        RunProcess();
        
    2: // 停止
        StopProcess();
        systemState := 0;
END_CASE;

END_PROGRAM
```

### 9.2 任务配置

```pascal
// 任务配置（通常在配置界面设置）
// 周期性任务 - 每10ms执行
Task_Main INTERVAL := T#10ms PRIORITY := 1;

// 事件触发任务
Task_Event EVENT := EventSignal PRIORITY := 2;

// 将程序分配给任务
MainControl WITH Task_Main;
AlarmHandler WITH Task_Event;
```

---

## 10. 数组与字符串处理

### 10.1 数组高级操作

```pascal
PROGRAM ArrayOperations
VAR
    // 动态数据处理
    sensorData : ARRAY[0..99] OF REAL;
    validCount : INT := 0;
    
    // 统计变量
    minValue : REAL;
    maxValue : REAL;
    averageValue : REAL;
    sumValue : REAL;
    
    i : INT;
END_VAR

// 查找最大值和最小值
minValue := 3.4E38;  // 最大浮点数
maxValue := -3.4E38; // 最小浮点数
sumValue := 0.0;

FOR i := 0 TO validCount - 1 DO
    IF sensorData[i] < minValue THEN
        minValue := sensorData[i];
    END_IF;
    
    IF sensorData[i] > maxValue THEN
        maxValue := sensorData[i];
    END_IF;
    
    sumValue := sumValue + sensorData[i];
END_FOR;

IF validCount > 0 THEN
    averageValue := sumValue / INT_TO_REAL(validCount);
END_IF;

END_PROGRAM
```

### 10.2 字符串处理

```pascal
PROGRAM StringHandling
VAR
    // 字符串变量
    deviceName : STRING := 'Motor_Controller_V1.0';
    statusText : STRING(100);
    
    // 处理结果
    nameLength : INT;
    versionStart : INT;
    versionString : STRING(10);
END_VAR

// 获取字符串长度
nameLength := LEN(deviceName);  // 23

// 查找子字符串
versionStart := FIND(deviceName, 'V');  // 16

// 提取子字符串
IF versionStart > 0 THEN
    versionString := MID(deviceName, versionStart, 10);
END_IF;

// 字符串拼接
statusText := CONCAT('设备: ', deviceName);
statusText := CONCAT(statusText, ' 状态: 运行中');

END_PROGRAM
```

---

## 11. 定时器与计数器

### 11.1 标准定时器

```pascal
PROGRAM TimerExamples
VAR
    // TON - 接通延时定时器
    tonTimer : TON;
    lightOn : BOOL;
    
    // TOF - 断开延时定时器
    tofTimer : TOF;
    fanRunning : BOOL;
    
    // TP - 脉冲定时器
    tpTimer : TP;
    pulseOutput : BOOL;
END_VAR

// TON 使用
tonTimer(IN := startSignal, PT := T#5s);
IF tonTimer.Q THEN
    lightOn := TRUE;
END_IF;

// TOF 使用
tofTimer(IN := runSignal, PT := T#10s);
fanRunning := tofTimer.Q;

// TP 使用
tpTimer(IN := triggerSignal, PT := T#1s);
pulseOutput := tpTimer.Q;

END_PROGRAM
```

### 11.2 计数器

```pascal
PROGRAM CounterExamples
VAR
    // CTU - 增计数器
    ctuCounter : CTU;
    productCount : DINT;
    
    // CTD - 减计数器
    ctdCounter : CTD;
    remainingItems : INT;
END_VAR

// CTU 使用
ctuCounter(CU := sensorSignal, 
           RESET := resetSignal,
           PV := 100);
productCount := ctuCounter.CV;
countReached := ctuCounter.Q;

END_PROGRAM
```

---

## 12. 自锁与互锁逻辑

自锁（Self-holding）和互锁（Interlock）是PLC编程中最基础也是最重要的控制逻辑，广泛应用于电机控制、安全保护、顺序控制等场景。

### 12.1 自锁电路（起保停电路）

自锁电路是最经典的PLC控制逻辑，实现"启动-保持-停止"功能。

#### 基本自锁电路

```pascal
PROGRAM BasicSelfHold
VAR_INPUT
    startButton : BOOL;     // 启动按钮（常开）
    stopButton : BOOL;      // 停止按钮（常闭）
END_VAR

VAR_OUTPUT
    motorRun : BOOL;        // 电机运行输出
END_VAR

// 经典自锁逻辑
motorRun := (startButton OR motorRun) AND NOT stopButton;

END_PROGRAM
```

**工作原理：**
- 按下启动按钮 → motorRun = TRUE
- 松开启动按钮 → motorRun 通过自身触点保持为 TRUE（自锁）
- 按下停止按钮 → motorRun = FALSE（解除自锁）

#### 带状态指示的自锁电路

```pascal
PROGRAM SelfHoldWithIndicator
VAR_INPUT
    startButton : BOOL;     // 启动按钮
    stopButton : BOOL;      // 停止按钮
    emergencyStop : BOOL;   // 急停按钮
END_VAR

VAR_OUTPUT
    motorRun : BOOL;        // 电机运行
    runIndicator : BOOL;    // 运行指示灯
    faultIndicator : BOOL;  // 故障指示灯
END_VAR

VAR
    motorFault : BOOL;      // 电机故障信号
    overloadRelay : BOOL;   // 过载继电器
END_VAR

// 自锁逻辑（包含多重保护）
IF emergencyStop OR overloadRelay OR motorFault THEN
    // 紧急停止或故障时立即停止
    motorRun := FALSE;
ELSIF stopButton THEN
    // 正常停止
    motorRun := FALSE;
ELSIF startButton THEN
    // 启动（只有在无故障状态下才能启动）
    IF NOT motorFault AND NOT overloadRelay THEN
        motorRun := TRUE;
    END_IF;
END_IF;

// 状态指示
runIndicator := motorRun;
faultIndicator := motorFault OR overloadRelay;

END_PROGRAM
```

#### 使用SET/RESET指令的自锁

```pascal
PROGRAM SetResetLatch
VAR_INPUT
    startButton : BOOL;
    stopButton : BOOL;
END_VAR

VAR_OUTPUT
    motorRun : BOOL;
END_VAR

// 使用SET/RESET指令（部分平台支持）
IF startButton THEN
    motorRun := TRUE;   // 或 SET(motorRun);
END_IF;

IF stopButton THEN
    motorRun := FALSE;  // 或 RESET(motorRun);
END_IF;

END_PROGRAM
```

### 12.2 互锁电路

互锁用于防止两个或多个设备同时运行，确保安全。

#### 基本互锁（正反转控制）

最经典的互锁应用是电机正反转控制，防止正转和反转接触器同时吸合造成短路。

```pascal
PROGRAM MotorForwardReverse
VAR_INPUT
    forwardBtn : BOOL;    // 正转按钮
    reverseBtn : BOOL;    // 反转按钮
    stopBtn : BOOL;       // 停止按钮
END_VAR

VAR_OUTPUT
    forwardRun : BOOL;    // 正转输出
    reverseRun : BOOL;    // 反转输出
END_VAR

VAR
    forwardLocked : BOOL; // 正转互锁标志
    reverseLocked : BOOL; // 反转互锁标志
END_VAR

// 正转控制（带互锁）
IF stopBtn THEN
    forwardRun := FALSE;
    reverseRun := FALSE;
ELSIF forwardBtn AND NOT reverseRun THEN
    // 只有在反转未运行时才能正转
    forwardRun := TRUE;
    reverseRun := FALSE;
END_IF;

// 反转控制（带互锁）
IF stopBtn THEN
    forwardRun := FALSE;
    reverseRun := FALSE;
ELSIF reverseBtn AND NOT forwardRun THEN
    // 只有在正转未运行时才能反转
    reverseRun := TRUE;
    forwardRun := FALSE;
END_IF;

END_PROGRAM
```

#### 三重互锁（更安全的正反转控制）

```pascal
PROGRAM TripleInterlock
VAR_INPUT
    forwardBtn : BOOL;
    reverseBtn : BOOL;
    stopBtn : BOOL;
END_VAR

VAR_OUTPUT
    forwardContact : BOOL;  // 正转接触器
    reverseContact : BOOL;  // 反转接触器
END_VAR

VAR
    forwardRequest : BOOL;  // 正转请求
    reverseRequest : BOOL;  // 反转请求
    changeoverDelay : TON;  // 换向延时定时器
END_VAR

// 请求信号（按钮触发）
IF forwardBtn THEN
    forwardRequest := TRUE;
    reverseRequest := FALSE;
ELSIF reverseBtn THEN
    reverseRequest := TRUE;
    forwardRequest := FALSE;
ELSIF stopBtn THEN
    forwardRequest := FALSE;
    reverseRequest := FALSE;
END_IF;

// 换向延时（防止快速切换）
changeoverDelay(IN := (forwardRequest XOR reverseRequest), 
                PT := T#500ms);

// 输出控制（三重互锁）
// 1. 电气互锁：NOT reverseContact
// 2. 机械互锁：changeoverDelay.Q
// 3. 软件互锁：NOT reverseRequest
forwardContact := forwardRequest 
                  AND NOT reverseContact 
                  AND changeoverDelay.Q
                  AND NOT reverseRequest;

reverseContact := reverseRequest 
                  AND NOT forwardContact 
                  AND changeoverDelay.Q
                  AND NOT forwardRequest;

END_PROGRAM
```

#### 多设备互锁（优先级控制）

```pascal
PROGRAM PriorityInterlock
VAR_INPUT
    pump1Start : BOOL;      // 泵1启动
    pump2Start : BOOL;      // 泵2启动
    pump3Start : BOOL;      // 泵3启动
    allStop : BOOL;         // 全部停止
END_VAR

VAR_OUTPUT
    pump1Run : BOOL;        // 泵1运行
    pump2Run : BOOL;        // 泵2运行
    pump3Run : BOOL;        // 泵3运行
END_VAR

VAR
    pump1Request : BOOL;
    pump2Request : BOOL;
    pump3Request : BOOL;
END_VAR

// 请求锁定
IF pump1Start THEN pump1Request := TRUE; END_IF;
IF pump2Start THEN pump2Request := TRUE; END_IF;
IF pump3Start THEN pump3Request := TRUE; END_IF;

IF allStop THEN
    pump1Request := FALSE;
    pump2Request := FALSE;
    pump3Request := FALSE;
END_IF;

// 优先级互锁（泵1 > 泵2 > 泵3）
// 只有当前面优先级的泵未运行时，后面的泵才能运行
IF pump1Request THEN
    pump1Run := TRUE;
    pump2Run := FALSE;
    pump3Run := FALSE;
ELSIF pump2Request AND NOT pump1Run THEN
    pump2Run := TRUE;
    pump1Run := FALSE;
    pump3Run := FALSE;
ELSIF pump3Request AND NOT pump1Run AND NOT pump2Run THEN
    pump3Run := TRUE;
    pump1Run := FALSE;
    pump2Run := FALSE;
ELSE
    pump1Run := FALSE;
    pump2Run := FALSE;
    pump3Run := FALSE;
END_IF;

END_PROGRAM
```

### 12.3 组合应用：自锁+互锁

实际工程中，自锁和互锁经常组合使用。

#### 多台电机顺序启动互锁

```pascal
PROGRAM SequentialMotorControl
VAR_INPUT
    startAll : BOOL;        // 全部启动
    stopAll : BOOL;         // 全部停止
    emergencyStop : BOOL;   // 急停
END_VAR

VAR_OUTPUT
    motor1Run : BOOL;       // 电机1（主电机）
    motor2Run : BOOL;       // 电机2（辅机1）
    motor3Run : BOOL;       // 电机3（辅机2）
END_VAR

VAR
    // 定时器
    delay1_2 : TON;         // 电机1到2的延时
    delay2_3 : TON;         // 电机2到3的延时
    
    // 中间状态
    motor1Started : BOOL;
    motor2Started : BOOL;
END_VAR

// 急停处理
IF emergencyStop THEN
    motor1Run := FALSE;
    motor2Run := FALSE;
    motor3Run := FALSE;
    motor1Started := FALSE;
    motor2Started := FALSE;
    RETURN;
END_IF;

// 停止处理
IF stopAll THEN
    motor1Run := FALSE;
    motor2Run := FALSE;
    motor3Run := FALSE;
    motor1Started := FALSE;
    motor2Started := FALSE;
END_IF;

// 电机1控制（自锁）
IF startAll AND NOT motor1Run THEN
    motor1Run := TRUE;
END_IF;

IF motor1Run THEN
    motor1Started := TRUE;
END_IF;

// 电机2控制（自锁+互锁）
// 互锁条件：电机1必须先运行
delay1_2(IN := motor1Started, PT := T#3s);

IF delay1_2.Q AND NOT motor2Run THEN
    motor2Run := TRUE;
END_IF;

IF motor2Run THEN
    motor2Started := TRUE;
END_IF;

// 电机3控制（自锁+互锁）
// 互锁条件：电机2必须先运行
delay2_3(IN := motor2Started, PT := T#2s);

IF delay2_3.Q AND NOT motor3Run THEN
    motor3Run := TRUE;
END_IF;

// 反向互锁：如果前面的电机停止，后面的也必须停止
IF NOT motor1Run THEN
    motor2Run := FALSE;
    motor3Run := FALSE;
END_IF;

IF NOT motor2Run THEN
    motor3Run := FALSE;
END_IF;

END_PROGRAM
```

#### 双工位选择互锁

```pascal
PROGRAM DualStationSelector
VAR_INPUT
    station1Select : BOOL;  // 选择工位1
    station2Select : BOOL;  // 选择工位2
    station1Start : BOOL;   // 工位1启动
    station2Start : BOOL;   // 工位2启动
    allStop : BOOL;         // 全部停止
END_VAR

VAR_OUTPUT
    station1Active : BOOL;  // 工位1激活
    station2Active : BOOL;  // 工位2激活
    station1Running : BOOL; // 工位1运行
    station2Running : BOOL; // 工位2运行
END_VAR

VAR
    station1Selected : BOOL;
    station2Selected : BOOL;
END_VAR

// 工位选择互锁（只能选择一个工位）
IF station1Select THEN
    station1Selected := TRUE;
    station2Selected := FALSE;
ELSIF station2Select THEN
    station2Selected := TRUE;
    station1Selected := FALSE;
END_IF;

// 工位激活显示
station1Active := station1Selected;
station2Active := station2Selected;

// 工位1运行控制（自锁+互锁）
IF allStop THEN
    station1Running := FALSE;
ELSIF station1Start AND station1Selected AND NOT station2Running THEN
    station1Running := TRUE;
END_IF;

// 工位2运行控制（自锁+互锁）
IF allStop THEN
    station2Running := FALSE;
ELSIF station2Start AND station2Selected AND NOT station1Running THEN
    station2Running := TRUE;
END_IF;

END_PROGRAM
```

### 12.4 高级互锁模式

#### 时间片轮转互锁

```pascal
PROGRAM TimeSlotInterlock
VAR_INPUT
    device1Request : BOOL;
    device2Request : BOOL;
    device3Request : BOOL;
END_VAR

VAR_OUTPUT
    device1Enable : BOOL;
    device2Enable : BOOL;
    device3Enable : BOOL;
END_VAR

VAR
    cycleTimer : TON;           // 周期定时器
    timeSlot : INT := 0;        // 当前时间片
    slotDuration : TIME := T#10s; // 每个时间片10秒
END_VAR

// 时间片循环
cycleTimer(IN := TRUE, PT := slotDuration);

IF cycleTimer.Q THEN
    cycleTimer(IN := FALSE);  // 复位
    timeSlot := timeSlot + 1;
    
    IF timeSlot > 2 THEN
        timeSlot := 0;
    END_IF;
END_IF;

// 根据时间片分配设备（互锁）
CASE timeSlot OF
    0:
        // 时间片0：设备1优先
        IF device1Request THEN
            device1Enable := TRUE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        ELSIF device2Request THEN
            device1Enable := FALSE;
            device2Enable := TRUE;
            device3Enable := FALSE;
        ELSIF device3Request THEN
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := TRUE;
        ELSE
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        END_IF;
        
    1:
        // 时间片1：设备2优先
        IF device2Request THEN
            device1Enable := FALSE;
            device2Enable := TRUE;
            device3Enable := FALSE;
        ELSIF device3Request THEN
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := TRUE;
        ELSIF device1Request THEN
            device1Enable := TRUE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        ELSE
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        END_IF;
        
    2:
        // 时间片2：设备3优先
        IF device3Request THEN
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := TRUE;
        ELSIF device1Request THEN
            device1Enable := TRUE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        ELSIF device2Request THEN
            device1Enable := FALSE;
            device2Enable := TRUE;
            device3Enable := FALSE;
        ELSE
            device1Enable := FALSE;
            device2Enable := FALSE;
            device3Enable := FALSE;
        END_IF;
END_CASE;

END_PROGRAM
```

#### 条件互锁（基于状态的互锁）

```pascal
PROGRAM ConditionalInterlock
VAR_INPUT
    valve1Open : BOOL;        // 阀门1打开命令
    valve2Open : BOOL;        // 阀门2打开命令
    tankLevel : REAL;         // 液位
    tankPressure : REAL;      // 压力
END_VAR

VAR_OUTPUT
    valve1Actual : BOOL;      // 阀门1实际状态
    valve2Actual : BOOL;      // 阀门2实际状态
    alarmHighLevel : BOOL;    // 高液位报警
    alarmHighPressure : BOOL; // 高压报警
END_VAR

VAR
    safetyInterlock : BOOL;   // 安全互锁
END_VAR

// 安全条件检查
alarmHighLevel := tankLevel > 90.0;
alarmHighPressure := tankPressure > 5.0;

// 安全互锁条件
safetyInterlock := NOT alarmHighLevel AND NOT alarmHighPressure;

// 阀门1控制（带条件互锁）
IF valve1Open AND safetyInterlock AND NOT valve2Actual THEN
    valve1Actual := TRUE;
ELSIF NOT valve1Open OR NOT safetyInterlock OR valve2Actual THEN
    valve1Actual := FALSE;
END_IF;

// 阀门2控制（带条件互锁）
IF valve2Open AND safetyInterlock AND NOT valve1Actual THEN
    valve2Actual := TRUE;
ELSIF NOT valve2Open OR NOT safetyInterlock OR valve1Actual THEN
    valve2Actual := FALSE;
END_IF;

END_PROGRAM
```

### 12.5 常见错误与注意事项

#### ❌ 常见错误1：忘记互锁导致冲突

```pascal
// 错误示例：没有互锁
IF button1 THEN output1 := TRUE; END_IF;
IF button2 THEN output2 := TRUE; END_IF;
// 问题：output1和output2可能同时为TRUE

// 正确示例：添加互锁
IF button1 AND NOT output2 THEN output1 := TRUE; END_IF;
IF button2 AND NOT output1 THEN output2 := TRUE; END_IF;
```

#### ❌ 常见错误2：自锁条件不完整

```pascal
// 错误示例：缺少停止条件
motorRun := startButton OR motorRun;  // 无法停止！

// 正确示例：完整的自锁
motorRun := (startButton OR motorRun) AND NOT stopButton;
```

#### ❌ 常见错误3：互锁逻辑不对称

```pascal
// 错误示例：不对称的互锁
IF cond1 THEN out1 := TRUE; END_IF;
IF cond2 AND NOT out1 THEN out2 := TRUE; END_IF;
// 问题：out1可以覆盖out2，但out2不能覆盖out1

// 正确示例：对称互锁
IF cond1 AND NOT out2 THEN out1 := TRUE; END_IF;
IF cond2 AND NOT out1 THEN out2 := TRUE; END_IF;
```

#### ✅ 最佳实践

1. **始终添加急停回路**
```pascal
IF emergencyStop THEN
    // 立即清除所有输出
    output1 := FALSE;
    output2 := FALSE;
    RETURN;
END_IF;
```

2. **使用中间变量提高可读性**
```pascal
// 清晰的状态判断
canStart := NOT fault AND NOT running AND ready;
IF startButton AND canStart THEN
    running := TRUE;
END_IF;
```

3. **添加状态反馈**
```pascal
// 不仅控制输出，还要监控状态
IF motorCommand AND NOT motorFeedback THEN
    // 命令已发出但没有反馈，可能是故障
    motorFault := TRUE;
END_IF;
```

4. **考虑扫描周期的影响**
```pascal
// 使用上升沿检测避免多次触发
IF risingEdge(startButton) THEN
    // 只在按钮按下的第一个扫描周期执行
    counter := counter + 1;
END_IF;
```

---

## 13. 实际工程案例

### 12.1 温度控制系统

```pascal
(*
    温度控制系统
    功能：PID控制加热器，实现精确温度控制
*)
PROGRAM TemperatureControl
VAR_INPUT
    setpoint : REAL := 100.0;    // 设定温度
    autoMode : BOOL;              // 自动模式
    manualPower : REAL;           // 手动功率(0-100%)
    resetSystem : BOOL;           // 系统复位
END_VAR

VAR_OUTPUT
    heaterPower : REAL;           // 加热器功率(0-100%)
    currentTemp : REAL;           // 当前温度
    systemStatus : INT;           // 系统状态
    alarmActive : BOOL;           // 报警激活
END_VAR

VAR
    // 温度传感器读数
    rawTemperature : REAL;
    filteredTemp : REAL;
    
    // PID控制器
    pidOutput : REAL;
    kp : REAL := 2.0;
    ki : REAL := 0.5;
    kd : REAL := 0.1;
    
    // 滤波器
    alpha : REAL := 0.2;  // 滤波系数
    
    // 安全限制
    maxTemp : REAL := 150.0;
    minTemp : REAL := 0.0;
    maxPower : REAL := 100.0;
    
    // 状态机
    controlState : INT := 0;
    
    // 定时器
    startupTimer : TON;
    
    // 统计
    overtempCount : INT := 0;
END_VAR

// 读取温度传感器（模拟）
rawTemperature := ReadTemperatureSensor();

// 低通滤波
filteredTemp := alpha * rawTemperature + (1.0 - alpha) * filteredTemp;
currentTemp := filteredTemp;

// 状态机
CASE controlState OF
    0: // 初始化
        heaterPower := 0.0;
        systemStatus := 0;
        startupTimer(IN := TRUE, PT := T#5s);
        
        IF startupTimer.Q THEN
            controlState := 1;
        END_IF;
        
    1: // 预热阶段
        systemStatus := 1;
        
        // 全功率加热到80%设定点
        IF currentTemp < (setpoint * 0.8) THEN
            heaterPower := 100.0;
        ELSE
            controlState := 2;  // 进入PID控制
        END_IF;
        
    2: // PID控制阶段
        systemStatus := 2;
        
        IF autoMode THEN
            // PID控制
            pidOutput := CalculatePID(setpoint, currentTemp, kp, ki, kd);
            
            // 限幅
            IF pidOutput > maxPower THEN
                heaterPower := maxPower;
            ELSIF pidOutput < 0.0 THEN
                heaterPower := 0.0;
            ELSE
                heaterPower := pidOutput;
            END_IF;
        ELSE
            // 手动模式
            heaterPower := LimitValue(manualPower, 0.0, maxPower);
        END_IF;
        
    3: // 报警状态
        systemStatus := 3;
        heaterPower := 0.0;
        alarmActive := TRUE;
        
        IF resetSystem THEN
            alarmActive := FALSE;
            overtempCount := 0;
            controlState := 0;
        END_IF;
END_CASE;

// 安全检查
IF currentTemp > maxTemp THEN
    overtempCount := overtempCount + 1;
    
    IF overtempCount > 10 THEN
        controlState := 3;  // 进入报警状态
    END_IF;
END_IF;

IF currentTemp < minTemp THEN
    // 低温报警
    alarmActive := TRUE;
END_IF;

// 复位处理
IF resetSystem AND controlState <> 3 THEN
    controlState := 0;
    overtempCount := 0;
END_IF;

END_PROGRAM
```

### 12.2 传送带控制系统

```pascal
(*
    传送带控制系统
    功能：多段传送带协调控制，产品计数和分拣
*)
PROGRAM ConveyorSystem
VAR_INPUT
    startCommand : BOOL;
    stopCommand : BOOL;
    emergencyStop : BOOL;
    productDetected : BOOL;  // 入口传感器
    sortSignal : BOOL;       // 分拣信号
END_VAR

VAR_OUTPUT
    conveyor1Run : BOOL;     // 传送带1
    conveyor2Run : BOOL;     // 传送带2
    diverterActive : BOOL;   // 分拣器
    totalCount : DINT;       // 总计数
    sortedCount : DINT;      // 分拣计数
    systemReady : BOOL;      // 系统就绪
END_VAR

VAR
    // 状态机
    systemState : INT := 0;
    
    // 产品跟踪
    productOnConveyor1 : BOOL;
    productOnConveyor2 : BOOL;
    
    // 定时器
    transferTimer : TON;
    diverterTimer : TON;
    
    // 传感器去抖
    sensorDebounce : TON;
    stableProductSignal : BOOL;
END_VAR

// 传感器去抖
sensorDebounce(IN := productDetected, PT := T#50ms);
stableProductSignal := sensorDebounce.Q;

// 状态机
CASE systemState OF
    0: // 待机状态
        conveyor1Run := FALSE;
        conveyor2Run := FALSE;
        diverterActive := FALSE;
        systemReady := TRUE;
        
        IF startCommand AND NOT emergencyStop THEN
            systemState := 1;
        END_IF;
        
    1: // 启动序列
        systemReady := FALSE;
        
        // 依次启动传送带
        conveyor1Run := TRUE;
        transferTimer(IN := TRUE, PT := T#2s);
        
        IF transferTimer.Q THEN
            conveyor2Run := TRUE;
            systemState := 2;
        END_IF;
        
    2: // 正常运行
        // 检测产品
        IF risingEdge(stableProductSignal) THEN
            productOnConveyor1 := TRUE;
            totalCount := totalCount + 1;
        END_IF;
        
        // 产品传输到传送带2
        IF productOnConveyor1 THEN
            transferTimer(IN := TRUE, PT := T#3s);
            
            IF transferTimer.Q THEN
                productOnConveyor1 := FALSE;
                productOnConveyor2 := TRUE;
            END_IF;
        END_IF;
        
        // 分拣控制
        IF productOnConveyor2 AND sortSignal THEN
            diverterActive := TRUE;
            sortedCount := sortedCount + 1;
            
            diverterTimer(IN := TRUE, PT := T#1s);
            IF diverterTimer.Q THEN
                diverterActive := FALSE;
                productOnConveyor2 := FALSE;
            END_IF;
        END_IF;
        
        // 停止命令
        IF stopCommand THEN
            systemState := 3;
        END_IF;
        
    3: // 停止序列
        // 等待传送带清空
        IF NOT productOnConveyor1 AND NOT productOnConveyor2 THEN
            conveyor1Run := FALSE;
            transferTimer(IN := TRUE, PT := T#2s);
            
            IF transferTimer.Q THEN
                conveyor2Run := FALSE;
                systemState := 0;
            END_IF;
        END_IF;
END_CASE;

// 紧急停止
IF emergencyStop THEN
    systemState := 4;
    conveyor1Run := FALSE;
    conveyor2Run := FALSE;
    diverterActive := FALSE;
END_IF;

END_PROGRAM
```

---

## 14. 最佳实践与调试技巧

### 14.1 编程最佳实践

#### 1. 代码组织结构
```pascal
// ✅ 推荐：清晰的分区
PROGRAM WellOrganized
VAR_INPUT
    // 输入信号
END_VAR

VAR_OUTPUT
    // 输出信号
END_VAR

VAR
    // 局部变量
END_VAR

// 主逻辑
// 1. 输入处理
// 2. 核心逻辑
// 3. 输出处理

END_PROGRAM
```

#### 2. 使用有意义的命名
```pascal
// ❌ 不推荐
a := b + c;

// ✅ 推荐
totalCost := unitPrice * quantity;
```

#### 3. 添加注释
```pascal
// 计算电机转速（RPM）
// 公式: RPM = (频率 * 60) / 极对数
motorRPM := (frequency * 60.0) / polePairs;
```

#### 4. 错误处理
```pascal
// 检查除零
IF divisor <> 0.0 THEN
    result := dividend / divisor;
ELSE
    result := 0.0;
    errorFlag := TRUE;
END_IF;

// 检查数组边界
IF index >= 0 AND index < ARRAY_SIZE THEN
    value := dataArray[index];
END_IF;
```

#### 5. 模块化设计
```pascal
// 将复杂逻辑分解为函数块
FB_PIDController
FB_MotorDriver
FB_SensorFilter
FB_AlarmManager
```

### 14.2 调试技巧

#### 1. 使用监视变量
```pascal
VAR DEBUG
    debugCounter : INT;
    debugValue : REAL;
    executionTime : TIME;
END_VAR
```

#### 2. 状态跟踪
```pascal
// 记录状态变化
IF currentState <> lastState THEN
    stateChangeTime := CURRENT_TIME();
    lastState := currentState;
END_IF;
```

#### 3. 性能监控
```pascal
// 测量扫描周期
VAR_STAT
    lastCycleTime : TIME;
    currentCycleTime : TIME;
END_VAR

currentCycleTime := CURRENT_TIME() - lastCycleTime;
lastCycleTime := CURRENT_TIME();

// 检查是否超时
IF currentCycleTime > T#10ms THEN
    performanceWarning := TRUE;
END_IF;
```

### 14.3 性能优化

#### 1. 避免不必要的计算
```pascal
// ❌ 每次都计算
FOR i := 0 TO 99 DO
    result[i] := value * SIN(2 * PI * i / 100);
END_FOR;

// ✅ 预先计算常量
CONSTANT_FACTOR := 2 * PI / 100;
FOR i := 0 TO 99 DO
    result[i] := value * SIN(CONSTANT_FACTOR * i);
END_FOR;
```

#### 2. 合理使用数据类型
```pascal
// 小范围计数使用 INT 而非 DINT
counter : INT;  // 足够且节省内存

// 精确计算使用 LREAL
preciseValue : LREAL;
```

---

## 15. 常见问题解答

### Q1: ST语言和其他PLC编程语言有什么区别？

**A:** 
- **vs 梯形图(LD)**: ST更适合复杂算法，LD更适合逻辑控制
- **vs 功能块图(FBD)**: ST更紧凑，FBD更直观
- **vs 顺序功能图(SFC)**: ST适合数据处理，SFC适合流程控制

### Q2: 如何处理浮点数精度问题？

**A:**
```pascal
// 使用容差比较
IF ABS(value1 - value2) < 0.001 THEN
    // 认为相等
END_IF;

// 使用ROUND函数
roundedValue := ROUND(preciseValue * 100.0) / 100.0;
```

### Q3: 如何实现掉电保持？

**A:**
```pascal
VAR RETAIN
    retainedValue : DINT;  // 掉电保持
END_VAR
```

### Q4: 如何优化扫描周期？

**A:**
- 减少不必要的计算
- 使用合适的数据类型
- 避免深层循环嵌套
- 将耗时操作分散到多个周期

### Q5: ST语言支持面向对象吗？

**A:** 
部分平台支持基本的OOP特性：
- 函数块可以有方法
- 支持继承（有限）
- 不支持多态

### Q6: 如何处理通信超时？

**A:**
```pascal
// 使用看门狗定时器
watchdogTimer(IN := communicationActive, PT := T#5s);

IF watchdogTimer.Q THEN
    // 超时处理
    ResetCommunication();
END_IF;
```

### Q7: 数组越界如何防止？

**A:**
```pascal
// 始终检查索引
IF index >= LOWER_BOUND(array) AND index <= UPPER_BOUND(array) THEN
    value := array[index];
END_IF;
```

### Q8: 如何实现状态机？

**A:** 使用 CASE 语句：
```pascal
CASE state OF
    STATE_IDLE:
        // 空闲处理
    STATE_RUNNING:
        // 运行处理
    STATE_ERROR:
        // 错误处理
END_CASE;
```

---

## 附录A: 常用标准函数

### 数学函数
```pascal
ABS(x)      // 绝对值
SQRT(x)     // 平方根
LN(x)       // 自然对数
LOG(x)      // 常用对数
EXP(x)      // 指数
SIN(x)      // 正弦
COS(x)      // 余弦
TAN(x)      // 正切
ASIN(x)     // 反正弦
ACOS(x)     // 反余弦
ATAN(x)     // 反正切
TRUNC(x)    // 截断
ROUND(x)    // 四舍五入
MIN(a,b)    // 最小值
MAX(a,b)    // 最大值
LIMIT(min,val,max)  // 限幅
```

### 字符串函数
```pascal
LEN(str)           // 字符串长度
LEFT(str,n)        // 左取n个字符
RIGHT(str,n)       // 右取n个字符
MID(str,start,len) // 取子串
CONCAT(str1,str2)  // 连接
FIND(str,substr)   // 查找
INSERT(str,pos,ins)// 插入
DELETE(str,pos,len)// 删除
REPLACE(str,pos,len,new) // 替换
```

### 转换函数
```pascal
TO_BOOL(x)
TO_INT(x)
TO_REAL(x)
TO_STRING(x)
INT_TO_REAL(x)
REAL_TO_INT(x)
STRING_TO_INT(x)
```

### 时间函数
```pascal
ADD(t1,t2)      // 时间相加
SUB(t1,t2)      // 时间相减
MUL(t,n)        // 时间乘
DIV(t,n)        // 时间除
CURRENT_TIME()  // 当前时间
GET_DATE()      // 获取日期
```

---

## 附录B: 快速参考卡片

### 基本结构
```pascal
PROGRAM ProgramName
VAR_INPUT ... END_VAR
VAR_OUTPUT ... END_VAR
VAR ... END_VAR
// 逻辑代码
END_PROGRAM

FUNCTION FuncName : ReturnType
VAR_INPUT ... END_VAR
FuncName := result;
END_FUNCTION

FUNCTION_BLOCK FB_Name
VAR_INPUT ... END_VAR
VAR_OUTPUT ... END_VAR
VAR ... END_VAR
// 逻辑代码
END_FUNCTION_BLOCK
```

### 控制结构速查
```pascal
// IF语句
IF condition THEN ... ELSIF ... ELSE ... END_IF;

// CASE语句
CASE value OF val1: ... val2: ... ELSE ... END_CASE;

// FOR循环
FOR i := start TO end BY step DO ... END_FOR;

// WHILE循环
WHILE condition DO ... END_WHILE;

// REPEAT循环
REPEAT ... UNTIL condition END_REPEAT;
```

### 数据类型速查
```pascal
BOOL       // 布尔
INT        // 16位整数
DINT       // 32位整数
REAL       // 32位浮点
LREAL      // 64位浮点
STRING     // 字符串
TIME       // 时间
DATE       // 日期
TOD        // 时刻
DT         // 日期时间
```

---

## 结语

恭喜您完成了ST语言的入门学习！记住：

1. **实践是最好的老师** - 多写代码，多做项目
2. **阅读文档** - 不同平台可能有差异
3. **向社区学习** - 参与论坛讨论
4. **持续改进** - 重构和优化你的代码

祝您在工业自动化领域取得成功！🎉

---

**参考资料：**
- IEC 61131-3 标准文档
- 各PLC厂商官方文档
- Codesys在线帮助
- PLCopen规范

**更新日期：** 2024-04-14  
**版本：** 1.0
