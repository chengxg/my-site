/**
 * 3D打印压力调平传感器, 应变片高速测量, web浏览器上位机, 波形实时刷新
 * 文件复制到 【天问block】 中编译,
 * 编译成功后使用【stc-isp】 下载到STC32g8k64芯片中, 运行主频设置为40M
 * @author b站-仁泉之子
 * @brief
 * @version 1
 * @date 2025-04-19
 *
 * @copyright Copyright (c) 2025
 */
#define IRC_40M
#define PLL_NO
#define boolean unsigned char
#define true 1
#define false 0
#define HIGH 1
#define LOW 0

#include <STC32G.h>
sbit ET2 = IE2 ^ 2;
sbit ET3 = IE2 ^ 5;
uint32 sys_clk = 40000000; // 40M主频 设置PWM、定时器、串口、EEPROM频率参数
#include <stdio.h>
#include "lib/rcclock.h"
#include "lib/UART.h"
#include "lib/ADC.h"
#include "lib/delay.h"
#include "lib/eeprom.h"

//========================================================================
// ---------------------- 串口发送接收部分 --------------------------------
//========================================================================
#define Uart1_ReceiveSize 128
#define Uart1_SendBufferSize 255
#define Uart1_SendDMABufferSize 64
#define Uart1_SendDMAGroupCount 16
#define Uart1_CommandBufferSize 255
uint8 Uart1_receiveBuffer[Uart1_ReceiveSize] = {0};
uint8 Uart1_sendGroupState[Uart1_SendDMAGroupCount] = {0}; // DMA空闲状态 0:空闲 1:等待发送 2:发送中
uint8 Uart1_sendGroupSize[Uart1_SendDMAGroupCount] = {0};  // DMA每组数据长度的数据长度, 最长255
uint8 Uart1_sendGroupOrder[Uart1_SendDMAGroupCount] = {0};
uint8 xdata Uart1_sendBuffer[Uart1_SendBufferSize] = {0};                                  // DMA按组发送顺序
uint8 xdata Uart1_sendDMABuffer[Uart1_SendDMAGroupCount][Uart1_SendDMABufferSize] = {{0}}; // DMA发送缓冲区
uint8 Uart1_commandBuffer[Uart1_CommandBufferSize] = {0};
uint8 UART1_isSend = 0;                // 是否正在发送数据
uint8 UART1_isDMASend = 0;             // 是否使用DMA发送
uint8 UART1_enable = 1;                // 是否启用串口
int8 Uart1_sendCurrentGroupIndex = -1; // 当前发送的组
uint32 UART1_sendCount = 0;            // 发送字节计数

void UART1_sendData();

// FIFO环形缓冲队列
typedef struct FIFOBuffer
{
  unsigned int headPos;    // 缓冲区头部位置
  unsigned int tailPos;    // 缓冲区尾部位置
  unsigned int bufferSize; // 缓冲区长度
  unsigned char *buffer;   // 缓冲区数组
};
struct FIFOBuffer uartReceiveFIFO;
struct FIFOBuffer uartSendFIFO;
struct FIFOBuffer uartSendGroupFIFO;

unsigned char FIFOBuffer_available(struct FIFOBuffer *fifo_buffer)
{
  return fifo_buffer->headPos != fifo_buffer->tailPos;
}

void FIFOBuffer_flush(struct FIFOBuffer *fifo_buffer)
{
  fifo_buffer->headPos = 0;
  fifo_buffer->tailPos = 0;
}

unsigned char FIFOBuffer_read(struct FIFOBuffer *fifo_buffer)
{
  unsigned char buf = 0;
  // 如果头尾接触表示缓冲区为空
  if (fifo_buffer->headPos != fifo_buffer->tailPos)
  {
    buf = fifo_buffer->buffer[fifo_buffer->headPos];
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize)
    {
      fifo_buffer->headPos = 0;
    }
  }
  return buf;
}

#define FIFOBUFFER_PUSH(fifo, buf)               \
  do                                             \
  {                                              \
    (fifo).buffer[(fifo).tailPos] = (buf);       \
    if (++(fifo).tailPos >= (fifo).bufferSize)   \
    {                                            \
      (fifo).tailPos = 0;                        \
    }                                            \
    if ((fifo).tailPos == (fifo).headPos)        \
    {                                            \
      if (++(fifo).headPos >= (fifo).bufferSize) \
      {                                          \
        (fifo).headPos = 0;                      \
      }                                          \
    }                                            \
  } while (0)

void FIFOBuffer_push(struct FIFOBuffer *fifo_buffer, unsigned char buf)
{
  fifo_buffer->buffer[fifo_buffer->tailPos] = buf; // 从尾部追加
  if (++fifo_buffer->tailPos >= fifo_buffer->bufferSize)
  { // 尾节点偏移
    fifo_buffer->tailPos = 0;
  }
  if (fifo_buffer->tailPos == fifo_buffer->headPos)
  {
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize)
    {
      fifo_buffer->headPos = 0;
    }
  }
}

// 自定义通信协议, 检测命令格式 {Start1,Start2,data...,dataLength,End1,End2}
struct MyCommand
{
  unsigned char Start1;
  unsigned char Start2;
  unsigned char End1;
  unsigned char End2;
  unsigned char isStart;
  unsigned char count;
  unsigned char bufferSize;
  unsigned char *buffer;

  // 检测命令格式 {Start1,Start2,data...,dataLength,End1,End2}
  void (*resolveCommandCallback)(unsigned char *buffer, unsigned char length);
};
struct MyCommand uartCommand;

unsigned char getCheckSum(unsigned char *buffer, unsigned int start, unsigned int end)
{
  unsigned int i = 0;
  unsigned char sum = 0;
  for (i = start; i < end; i++)
  {
    sum += buffer[i];
  }
  return sum;
}
void MyCommand_addData(struct MyCommand *command, unsigned char tempData)
{
  if (!command->isStart)
  {
    if (command->count == 0 && tempData != command->Start1)
    {
      return;
    }
    command->count++;
    if (command->count == 2)
    {
      if (command->Start2 == tempData)
      {
        command->isStart = true;
        command->count = 0;
      }
      else
      {
        command->count = 0;
        if (tempData == command->Start1)
        {
          command->count++;
        }
      }
    }
    if (command->count > 2)
    {
      command->count = 0;
      command->isStart = false;
    }
    return;
  }

  if (command->count >= command->bufferSize)
  {
    command->count = 0;
    command->isStart = false;
  }

  command->buffer[command->count] = tempData;
  command->count++;

  if (command->isStart && command->count >= 4)
  {
    // 检测结束
    if (tempData == command->End2 &&
        command->buffer[command->count - 2] == command->End1)
    {
      // 长度位
      if (command->buffer[command->count - 3] == command->count - 3)
      {
        if (command->resolveCommandCallback)
        {
          command->resolveCommandCallback(command->buffer, command->count - 3);
        }
        command->isStart = false;
        command->count = 0;
      }
    }
  }
}
void UART1_Isr(void) interrupt 4
{
  if (TI)
  {
    TI = 0;
    // 队列中还有数据, 继续发送
    if (uartSendFIFO.headPos != uartSendFIFO.tailPos)
    {
      SBUF = FIFOBuffer_read(&uartSendFIFO);
    }
    else
    {
      UART1_isSend = 0;
    }
  }
  if (RI)
  {
    RI = 0;
    // 接收到数据, 放入队列
    // FIFOBuffer_push(&uartReceiveFIFO, SBUF);
    FIFOBUFFER_PUSH(uartReceiveFIFO, SBUF);
  }
}
void UART1_initCommand(void (*commandCallback)(uint8 *buffer, uint8 length))
{
  ES = 1; // 允许串行口中断

  uartReceiveFIFO.headPos = 0;
  uartReceiveFIFO.tailPos = 0;
  uartReceiveFIFO.bufferSize = Uart1_ReceiveSize;
  uartReceiveFIFO.buffer = Uart1_receiveBuffer;

  uartSendFIFO.headPos = 0;
  uartSendFIFO.tailPos = 0;
  uartSendFIFO.bufferSize = Uart1_SendBufferSize;
  uartSendFIFO.buffer = Uart1_sendBuffer;

  // DMA按组发送
  uartSendGroupFIFO.headPos = 0;
  uartSendGroupFIFO.tailPos = 0;
  uartSendGroupFIFO.bufferSize = Uart1_SendDMAGroupCount;
  uartSendGroupFIFO.buffer = Uart1_sendGroupOrder;

  uartCommand.Start1 = 0xf2;
  uartCommand.Start2 = 0xf3;
  uartCommand.End1 = 0xe2;
  uartCommand.End2 = 0xe3;
  uartCommand.isStart = 0;
  uartCommand.count = 0;
  uartCommand.bufferSize = Uart1_CommandBufferSize;
  uartCommand.buffer = Uart1_commandBuffer;
  uartCommand.resolveCommandCallback = commandCallback;
}
uint8 UART1_getIdleGroupIndex()
{
  uint8 i = 0;
  for (i = 0; i < Uart1_SendDMAGroupCount; i++)
  {
    if (Uart1_sendGroupState[i] == 0)
    {
      return i;
    }
  }
  return -1;
}
void UART1_DMA_Config(uint8 groupIndex)
{
  UART1_sendCount++;
  DMA_UR1T_CFG = 0x80; // bit7 1:Enable Interrupt
  DMA_UR1T_STA = 0x00;
  DMA_UR1T_AMT = Uart1_sendGroupSize[groupIndex] - 1;                     // 设置传输总字节数(低8位)：n+1
  DMA_UR1T_AMTH = 0x00;                                                   // 设置传输总字节数(高8位)：n+1
  DMA_UR1T_TXAH = (uint8)((uint16)&Uart1_sendDMABuffer[groupIndex] >> 8); // 缓冲区地址
  DMA_UR1T_TXAL = (uint8)((uint16)&Uart1_sendDMABuffer[groupIndex]);
  DMA_UR1T_CR = 0xc0; // bit7 1:使能 UART1_DMA, bit6 1:开始 UART1_DMA 自动发送
}
void UART1_writeBuffer(uint8 *buffer, uint8 length)
{
  uint8 i = 0;
  if (UART1_enable == 0)
  {
    return;
  }
  for (i = 0; i < length; i++)
  {
    FIFOBuffer_push(&uartSendFIFO, buffer[i]);
  }
  if (UART1_isSend == 0 && UART1_isDMASend == 0)
  {
    UART1_isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
  }
}
void UART1_writeBufferByDMA(uint8 groupIndex, uint8 length)
{
  if (UART1_enable == 0)
  {
    return;
  }
  Uart1_sendGroupSize[groupIndex] = length;
  if (UART1_isSend == 0 && UART1_isDMASend == 0)
  {
    UART1_isDMASend = 1;
    Uart1_sendCurrentGroupIndex = groupIndex;
    Uart1_sendGroupState[groupIndex] = 2;
    UART1_DMA_Config(groupIndex);
  }
  else
  {
    Uart1_sendGroupState[groupIndex] = 1;
    FIFOBuffer_push(&uartSendGroupFIFO, groupIndex);
  }
}
void UART1_DMA_Interrupt(void) interrupt 50
{
  if (DMA_UR1T_STA & 0x01) // 发送完成
  {
    DMA_UR1T_STA &= ~0x01;
    if (Uart1_sendCurrentGroupIndex > -1)
    {
      Uart1_sendGroupState[Uart1_sendCurrentGroupIndex] = 0; // 发送完成, 设置空闲
    }
    UART1_isDMASend = 0;
    Uart1_sendCurrentGroupIndex = -1;

    // 继续发送下一帧数据
    if ((uartSendGroupFIFO.headPos != uartSendGroupFIFO.tailPos))
    {
      Uart1_sendCurrentGroupIndex = FIFOBuffer_read(&uartSendGroupFIFO);
      if (Uart1_sendGroupSize[Uart1_sendCurrentGroupIndex] > 0)
      {
        UART1_isDMASend = 1;
        Uart1_sendGroupState[Uart1_sendCurrentGroupIndex] = 2;
        UART1_DMA_Config(Uart1_sendCurrentGroupIndex);
      }
      else
      {
        Uart1_sendGroupState[Uart1_sendCurrentGroupIndex] = 0;
        Uart1_sendCurrentGroupIndex = -1;
        UART1_isDMASend = 0;
      }
    }
  }
  if (DMA_UR1T_STA & 0x04) // 数据覆盖
  {
    DMA_UR1T_STA &= ~0x04;
  }
}
void UART1_sendData()
{
  if (UART1_enable == 0)
  {
    return;
  }
  // 中断发送数据
  if (UART1_isSend == 0 && UART1_isDMASend == 0 && uartSendFIFO.headPos != uartSendFIFO.tailPos)
  {
    UART1_isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
    return;
  }
  // DMA发送数据
  while (UART1_isSend == 0 && UART1_isDMASend == 0 && (uartSendGroupFIFO.headPos != uartSendGroupFIFO.tailPos))
  {
    Uart1_sendCurrentGroupIndex = FIFOBuffer_read(&uartSendGroupFIFO);
    if (Uart1_sendGroupSize[Uart1_sendCurrentGroupIndex] > 0)
    {
      UART1_isDMASend = 1;
      Uart1_sendGroupState[Uart1_sendCurrentGroupIndex] = 2;
      UART1_DMA_Config(Uart1_sendCurrentGroupIndex);
      break;
    }
    else
    {
      Uart1_sendGroupState[Uart1_sendCurrentGroupIndex] = 0;
      Uart1_sendCurrentGroupIndex = -1;
      UART1_isDMASend = 0;
    }
  }
}
//========================================================================
// ---------------------- 串口发送接收部分  结束----------------------------
//========================================================================

//========================================================================
// ---------------------- 类javascript定时器  开始-------------------------
//========================================================================
#define JSTimeSize 15 // 初始化定时的个数, 数量越多, 运行效率越低, 够用就好

unsigned long globeTime = 0;     // 定时器0溢出次数
unsigned long system_us = 0;     // 系统微秒计时
unsigned long tempGlobeTime = 0; // 临时存储globeTime
unsigned char tempTH0 = 0;       // 临时存储TH0
unsigned char tempTL0 = 0;       // 临时存储TL0
struct JSTimeStruct
{
  unsigned long id;         // 定时器id,用来取消定时器, id>0x80000000是setTimeout的id, id<0x80000000是setInterval的id
  unsigned long startTime;  // 开始执行的时间, 单位us
  unsigned long periodTime; // 延时的时间 或者 间隔执行的时间, 单位us
  void (*callback)();       // 回调函数
};
struct JSTimeStruct JSTime_arr[JSTimeSize];
// 获取系统微秒计时
unsigned long micros()
{
  // 系统微秒计时, 保持严格单调递增, 禁止出现时间倒退, 32位溢出时间为71分钟
  // globeTime*65536+(TH0-0)*256+(TL0-0)
  do
  {
    // 如果在读取过程中发生溢出，重新读取
    tempTH0 = TH0;
    tempTL0 = TL0;
    tempGlobeTime = globeTime;
  } while (tempTH0 != TH0 || tempTL0 != TL0 || tempGlobeTime != globeTime);

  // 计算当前时间
  system_us = (tempGlobeTime << 16) + (tempTH0 << 8 | tempTL0);
  return system_us;
}
// 定时器0中断
void T_IRQ0(void) interrupt 1 using 1
{
  globeTime++; // 定时器0溢出次数
}
// 定时器0初始化
void Timer0_Init(void) // 65536微秒@40.000MHz
{
  TM0PS = 0x27; // 设置定时器时钟预分频
  AUXR |= 0x80; // 定时器时钟1T模式
  TMOD &= 0xF0; // 设置定时器模式
  TL0 = 0x00;   // 设置定时初始值
  TH0 = 0x00;   // 设置定时初始值
  TF0 = 0;      // 清除TF0标志
  TR0 = 1;      // 定时器0开始计时
  ET0 = 1;      // 打开定时器0中断
  IPH |= 0x02;  // 设置定时器0的中断的优先级为高
  IP |= 0x02;   // 设置定时器0的中断的优先级为高
}
// 定时器初始化函数
void JSTime_init()
{
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++)
  {
    JSTime_arr[i].callback = 0;
    JSTime_arr[i].periodTime = 0;
    JSTime_arr[i].startTime = 0;
    JSTime_arr[i].id = 0;
  }
  Timer0_Init();
}
unsigned long JSTime_createTimeId = 0x80000000; // `setTimeout`的id
unsigned long JSTime_createIntervalTimeId = 1;  // `setInterval`的id
// loop循环中不断刷新定时器, JSTime_refresh中的局部变量变成全局变量
unsigned long JSTime_cacheId = 0; // 缓存id
unsigned long JSTime_currentTime = 0;
unsigned char JSTime_i = 0;
unsigned char JSTime_isFree = 0;
void JSTime_refresh()
{
  JSTime_currentTime = micros();
  for (JSTime_i = 0; JSTime_i < JSTimeSize; JSTime_i++)
  {
    if (JSTime_arr[JSTime_i].id != 0)
    {
      if (JSTime_currentTime - JSTime_arr[JSTime_i].startTime >=
          JSTime_arr[JSTime_i].periodTime)
      {
        JSTime_cacheId = JSTime_arr[JSTime_i].id;
        if (JSTime_cacheId >= 0x80000000)
        {
          // setTimeout 执行完毕就销毁
          JSTime_isFree = 1;
        }
        else
        {
          JSTime_isFree = 0;
          // setInteval 不断进行
          JSTime_arr[JSTime_i].startTime = JSTime_currentTime;
        }
        if (JSTime_arr[JSTime_i].callback)
        {
          JSTime_arr[JSTime_i].callback();
          JSTime_currentTime = micros();
        }
        // 防止在回调函数里调用了 clearTime 而引发bug
        if (JSTime_isFree == 1 && JSTime_arr[JSTime_i].id == JSTime_cacheId)
        {
          // setTimeout 执行完毕就销毁
          JSTime_arr[JSTime_i].id = 0;
        }
      }
    }
  }
}
// 延时执行, delayTime单位为us
unsigned long setTimeout_us(void (*callback)(), uint32 delayTime)
{
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++)
  {
    // 找出失效的 结构体
    if (JSTime_arr[i].id == 0)
    {
      JSTime_arr[i].callback = callback;
      JSTime_arr[i].periodTime = delayTime;
      JSTime_arr[i].startTime = micros();
      if (JSTime_createTimeId > 0xfffffff0)
      {
        JSTime_createTimeId = 0x80001000;
      }
      JSTime_createTimeId++;
      JSTime_arr[i].id = JSTime_createTimeId;
      return JSTime_createTimeId;
    }
  }
  return 0;
}
// 延时执行, delayTime单位为ms
unsigned long setTimeout(void (*callback)(), uint32 delayTime)
{
  return setTimeout_us(callback, delayTime * 1000);
}
// 间隔时间执行, intervalTime单位为us
unsigned long setInterval_us(void (*callback)(), uint32 intervalTime)
{
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++)
  {
    // 找出失效的 结构体
    if (JSTime_arr[i].id == 0)
    {
      JSTime_arr[i].startTime = micros();
      JSTime_arr[i].callback = callback;
      JSTime_arr[i].periodTime = intervalTime;
      if (JSTime_createIntervalTimeId > 0x7ffffff0)
      {
        JSTime_createIntervalTimeId = 1000;
      }
      JSTime_createIntervalTimeId++;
      JSTime_arr[i].id = JSTime_createIntervalTimeId;
      return JSTime_createIntervalTimeId;
    }
  }
  return 0;
}
// 间隔时间执行, intervalTime单位为ms
unsigned long setInterval(void (*callback)(), uint32 intervalTime)
{
  return setInterval_us(callback, intervalTime * 1000);
}
// 停止软件定时器计时
void clearTime(unsigned long timeId)
{
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++)
  {
    if (timeId == JSTime_arr[i].id)
    {
      JSTime_arr[i].id = 0;
    }
  }
}
// 停止所有软件定时器计时
void clearAllTime()
{
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++)
  {
    JSTime_arr[i].id = 0;
  }
}
//========================================================================
// ---------------------- 类javascript定时器  结束-------------------------
//========================================================================

//========================================================================
// ---------------------- 通用按钮  开始----------------------------
//========================================================================
unsigned long MyBtn_longPressInterval = 400 * 1000;        // 长按间隔时间, 每过一段时间触发一次长按回调
unsigned long MyBtn_longPressTriggerTime = 1500 * 1000;    // 检测长按触发时间
unsigned long MyBtn_doubleTriggerTime = 800 * 1000;        // 双击触发时间, 在这个时间内完成点击2次
unsigned long MyBtn_refreshDoubleTriggerTime = 400 * 1000; // 重新检测双击时间
unsigned long MyBtn_debounceTime = 20 * 1000;              // 按键防抖延时时间

struct MyBtn
{
  unsigned long btnTempDownStartTime;    // 开始按下的时间
  unsigned long btnTempUpStartTime;      // 开始抬起的时间
  unsigned long btnDownStartTime;        // 开始按下的时间
  unsigned long btnUpStartTime;          // 开始抬起的时间
  unsigned long btnLongPressLastTime;    // 上次长按触发的时间
  unsigned long btnDoubleClickStartTime; // 双击开始时间

  void (*keydownCallback)();          // 按键按下回调
  void (*keyupCallback)();            // 按键抬起回调
  void (*longPressCallback)();        // 长按回调
  void (*doubleClickCallback)();      // 双击回调
  unsigned char clickCount;           // 按键计数
  unsigned char isKeydown;            // 是否按下
  unsigned char isLongPressTriggered; // 长按是否已经触发过了

  unsigned char enableLongPressMultipleTrigger; // 长按时是否启用多次连续触发
};

// 按键初始化
void MyBtn_init(struct MyBtn *myBtn)
{
  myBtn->btnTempDownStartTime = 0;
  myBtn->btnTempUpStartTime = 0;
  myBtn->btnDownStartTime = 0;
  myBtn->btnUpStartTime = 0;
  myBtn->btnLongPressLastTime = 0;
  myBtn->btnDoubleClickStartTime = 0;
  myBtn->keydownCallback = NULL;
  myBtn->keyupCallback = NULL;
  myBtn->longPressCallback = NULL;
  myBtn->doubleClickCallback = NULL;
  myBtn->clickCount = 0;
  myBtn->isKeydown = false;
  myBtn->isLongPressTriggered = false;
  myBtn->enableLongPressMultipleTrigger = false;
}

// 检测按键状态
void MyBtn_check(struct MyBtn *myBtn, unsigned char isActive)
{
  // 按下
  if (isActive && !myBtn->isKeydown)
  {
    if (myBtn->btnTempDownStartTime == 0)
    {
      myBtn->btnTempDownStartTime = system_us;
    }
    // 延时防抖
    if (system_us - myBtn->btnTempDownStartTime > MyBtn_debounceTime)
    {
      myBtn->btnTempDownStartTime = 0;
      myBtn->btnDownStartTime = system_us;
      myBtn->isKeydown = true;
      // 距离上次单击时间超长, 重新检测双击
      if (system_us - myBtn->btnDoubleClickStartTime > MyBtn_refreshDoubleTriggerTime)
      {
        myBtn->clickCount = 0;
      }
      myBtn->clickCount++;
      // 记录双击开始时间
      if (myBtn->clickCount == 1)
      {
        myBtn->btnDoubleClickStartTime = system_us;
      }
      if (myBtn->keydownCallback != NULL)
      {
        myBtn->keydownCallback();
      }
    }
    return;
  }

  // 长按
  if (isActive && myBtn->isKeydown && myBtn->longPressCallback != NULL)
  {
    // 达到长按触发时间, 进入长按
    if (system_us - myBtn->btnDownStartTime > MyBtn_longPressTriggerTime)
    {
      if (myBtn->enableLongPressMultipleTrigger)
      {
        // 每隔MyBtn_longPressInterval, 自动触发一次长按事件
        if (system_us - myBtn->btnLongPressLastTime > MyBtn_longPressInterval)
        {
          myBtn->btnLongPressLastTime = system_us;
          myBtn->isLongPressTriggered = true;
          myBtn->longPressCallback();
        }
      }
      else
      {
        // 只触发一次长按事件
        if (myBtn->isLongPressTriggered == false)
        {
          myBtn->btnLongPressLastTime = 0;
          myBtn->isLongPressTriggered = true;
          myBtn->longPressCallback();
        }
      }
    }
    else
    {
      myBtn->btnLongPressLastTime = 0;
    }
    return;
  }

  // 抬起
  if (!isActive && myBtn->isKeydown)
  {
    if (myBtn->btnTempUpStartTime == 0)
    {
      myBtn->btnTempUpStartTime = system_us;
    }
    // 延时20ms, 防抖
    if (system_us - myBtn->btnTempUpStartTime > MyBtn_debounceTime)
    {
      myBtn->btnTempUpStartTime = 0;
      myBtn->btnLongPressLastTime = 0;
      myBtn->isKeydown = false;

      // 如果一次单击时间超长,那么双击无效
      if (system_us - myBtn->btnDoubleClickStartTime > MyBtn_refreshDoubleTriggerTime)
      {
        myBtn->clickCount = 0;
      }

      // 第二次单击抬起, 检测是否为双击
      if (myBtn->clickCount == 2)
      {
        myBtn->clickCount = 0;
        if (myBtn->doubleClickCallback != NULL &&
            system_us - myBtn->btnDoubleClickStartTime < MyBtn_doubleTriggerTime)
        {
          myBtn->doubleClickCallback();
          myBtn->isLongPressTriggered = false;
          return;
        }
      }

      // 单击抬起
      if (myBtn->keyupCallback != NULL && myBtn->isLongPressTriggered == false)
      {
        myBtn->keyupCallback();
      }
      myBtn->isLongPressTriggered = false;
    }
    return;
  }
}
//========================================================================
// ---------------------- 通用按钮  结束----------------------------
//========================================================================

uint8 version = 0x01;    // 程序版本号
uint32 loopCount = 0;    // 程序循环次数
uint32 lastLoopTime = 0; // 上次检测时间
uint32 maxLoopTime = 0;  // 单次最大循环间隔时间
uint32 debugInt1 = 0;    // 测试数据1
uint32 debugInt2 = 0;    // 测试数据2

uint8 cmdData[64] = {0};
uint32 autoSampleADCTimeoutId = 0; // 超过一定时间未接收到信号就自动停止发送
uint32 EEPROMSaveTimeoutId = 0;    // EEPROM延时保存配置的定时器id
uint32 uart1Baud = 2000000;        // 串口1波特率, 默认2M
uint32 changeBaudTimeoutId = 0;    // 改变波特率超时的定时器id

// 运算放大器方案 引脚定义
#define PROBE_OUT_PIN P3_5                 // 信号输出
#define PROBE_OUT2_PIN P3_4                // 信号输出 反相
#define LED_PIN P1_1                       // 状态指示灯
#define LED2_PIN P3_7                      // 信号输出指示灯
#define BTN_PIN P1_0                       // 按钮
#define ADC1_PIN ADC_P14                   // ADC通道1
#define ADC2_PIN ADC_P15                   // ADC通道2
uint16 sampleChannel = 0;                  // 当前通道
uint8 adcPolarity = 1;                     // ADC极性 1:正信号 0:负信号
uint16 adcValue = 0;                       // 当前adc读数
uint16 adcValue1 = 0;                      // 当前adc读数
uint16 adcValue2 = 0;                      // 当前adc读数
uint16 trigThreshold = 60;                 // 信号输出触发阈值
uint8 trigDirection = 1;                   // 信号输出触发方向 1:大于阈值触发 0:小于阈值触发
uint8 outputDirection = 0;                 // 输出方向 1:触发时输出高电平 0:触发时输出低电平
int16 compareValue = 0;                    // adc比较值
int16 compareValueLOW = 0;                 // adc比较值
uint16 sampleTime = 0;                     // 上位机显示数据 采样时间, us
uint16 sampleTimeCount = 0;                // 上位机显示数据 采样时间计数, 每获取一次adc数据, 10us一次
uint16 sampleTimeCurrentCount = 0;         // 上位机显示数据 采样时间计数, 当前计数, 和sampleTimeCount比较
#define ADC_CH 1                           /* 1~16, ADC转换通道数, 需同步修改 DMA_ADC_CHSW 转换通道 */
#define ADC_DATA 68                        /* 6~n, 每个通道ADC转换数据总数, 2*转换次数+4, 需同步修改 DMA_ADC_CFG2 转换次数 */
uint8 xdata ADC_DMABuffer[ADC_DATA] = {0}; // ADC DMA缓存
#define ADCCacheLength 100
uint16 adc1Array[ADCCacheLength] = {0};
uint8 adcCurrentIndex = 0;                                // 当前ADC添加的缓存索引
uint16 calibrationError = 25;                             // 一定时间内检测最大值和最小值范围, 小于这个就自动校准
uint16 calibrationMaxValue = 0;                           // 校准最大值
uint16 calibrationMinValue = 65535;                       // 校准最小值
uint32 calibrationSumValue = 0;                           // 校准和
uint32 calibrationCount = 0;                              // 校准sum数据个数
uint32 calibrationStartTime = 0;                          // 校准开始时间
uint8 btnIsSetting = 0;                                   // 是否正在设置
uint16 btnTrigThresholdArray[5] = {20, 40, 60, 80, 100};  // 按钮档位调节, 触发值
uint8 btnCalibrationErrorArray[5] = {18, 20, 25, 25, 30}; // 按钮档位调节, 校准误差
uint8 enableActivatePin = 0;                              // klipper probe 是否使能激活引脚
uint16 activateProtectValue = 0;                          // klipper probe 不使能时的极限保护值, 要比trigThreshold大

void setLed1State(uint8 led1State);

//========================================================================
// ---------------------- 运算放大器调平方案 开始 ------------------------------
//========================================================================

// DMA自动采集ADC数据
void DMA_ADC_Config(void)
{
  // CSSETUP CSHOLD[1:0] SMPDUTY[4:0]
  ADCTIM = 0x2f; // 设置通道选择时间、保持时间、采样时间 0b00101111
  ADCCFG = 0x20; // - - RESFMT - SPEED[3:0]
  // ADC_POWER ADC_START ADC_FLAG ADC_EPWMT ADC_CHS[3:0]
  ADC_CONTR = 0x80;

  DMA_ADC_STA = 0x00;
  DMA_ADC_CFG = 0x00;                                     // bit7 不使能DMA中断
  DMA_ADC_RXAH = (uint8)((uint16)&ADC_DMABuffer[0] >> 8); // ADC转换数据存储地址
  DMA_ADC_RXAL = (uint8)((uint16)&ADC_DMABuffer[0]);
  DMA_ADC_CFG2 = 0x00;  // 每个通道ADC转换次数: 1次 0
  DMA_ADC_CHSW0 = 0x10; // ADC通道使能寄存器 ADC7~ADC0 0b0011 0000
  DMA_ADC_CHSW1 = 0x00; // ADC通道使能寄存器 ADC15~ADC8
  adcPolarity = 1;      // 正相
  // DMA_ADC_CR = 0xc0;    // bit7 1:Enable ADC_DMA, bit6 1:Start ADC_DMA
}
// 定时器2定时触发ADC采集
void Timer2_Init(void) // 5微秒@40.000MHz
{
  TM2PS = 0x00; // 设置定时器时钟预分频 ( 注意:并非所有系列都有此寄存器,详情请查看数据手册 )
  AUXR |= 0x04; // 定时器时钟1T模式
  T2L = 0x38;   // 设置定时初始值
  T2H = 0xFF;   // 设置定时初始值
  AUXR |= 0x10; // 定时器2开始计时
  ET2 = 1;      // 使能定时器中断
}

// 定时器2中断
void TM2_Isr() interrupt 12
{
  adcValue = ADC_DMABuffer[0];
  adcValue <<= 8;
  adcValue |= ADC_DMABuffer[1];
  if (adcValue < 50)
  {
    // 自动切换通道
    if (DMA_ADC_CHSW0 == 0x10)
    {
      DMA_ADC_CHSW0 = 0x20;
      adcPolarity = 0; // 反相
    }
    else
    {
      DMA_ADC_CHSW0 = 0x10;
      adcPolarity = 1; // 正相
    }
  }
  DMA_ADC_CR = 0xc0; // bit7 1:Enable ADC_DMA, bit6 1:Start ADC_DMA

  // 当前达到触发值
  // 高于比较值触发
  if (trigDirection == 1)
  {
    if (adcValue > compareValue || adcValue < 50 || adcValue > 4000)
    {
      PROBE_OUT_PIN = outputDirection;
      LED2_PIN = !btnIsSetting; // 按钮设置状态的时候不亮
    }
    else if (adcValue < compareValue - 6)
    {
      PROBE_OUT_PIN = !outputDirection;
      LED2_PIN = LOW;
    }
  }
  else
  {
    if (adcValue < compareValue || adcValue < 50 || adcValue > 4000)
    {
      PROBE_OUT_PIN = outputDirection;
      LED2_PIN = !btnIsSetting; // 按钮设置状态的时候不亮
    }
    else if (adcValue > compareValue + 6)
    {
      PROBE_OUT_PIN = !outputDirection;
      LED2_PIN = LOW;
    }
  }

  // 添加采样数据
  if (sampleTimeCount > 0)
  {
    sampleTimeCurrentCount++;
    if (sampleTimeCurrentCount >= sampleTimeCount)
    {
      sampleTimeCurrentCount = 0;
      adc1Array[adcCurrentIndex] = adcValue;
      adcCurrentIndex++;
      if (adcCurrentIndex >= ADCCacheLength)
      {
        adcCurrentIndex = 0;
      }
    }
  }
}
// 发送ADC数据
void sendADCSampleSimple()
{
  uint8 i = 0;
  uint8 j = 0;
  uint8 size = adcCurrentIndex;
  uint8 *buffer = NULL;     // DMA缓存
  int8 dmaBufferIndex = -1; // DMA缓存的索引
  dmaBufferIndex = UART1_getIdleGroupIndex();
  if (dmaBufferIndex == -1)
  {
    return;
  }
  buffer = &Uart1_sendDMABuffer[dmaBufferIndex][0];
  i = 0;
  buffer[i++] = uartCommand.Start1;
  buffer[i++] = uartCommand.Start2;
  buffer[i++] = 0x20;
  buffer[i++] = size;              // 数据个数
  buffer[i++] = compareValue >> 8; // 触发值
  buffer[i++] = compareValue;
  // 拼接各个通道的数据
  for (j = 0; j < size; j++)
  {
    buffer[i++] = adc1Array[j] >> 8;
    buffer[i++] = adc1Array[j] & 0x00ff;
  }
  buffer[i++] = getCheckSum(buffer, 2, i); // 校验和
  buffer[i++] = i - 2;
  buffer[i++] = uartCommand.End1;
  buffer[i++] = uartCommand.End2;
  UART1_writeBufferByDMA(dmaBufferIndex, i);
}

void autoInitADC()
{
  if (adcValue > calibrationMaxValue)
  {
    calibrationMaxValue = adcValue;
  }
  if (adcValue < calibrationMinValue)
  {
    calibrationMinValue = adcValue;
  }
  calibrationSumValue += adcValue;
  calibrationCount++;

  if (calibrationMaxValue - calibrationMinValue > calibrationError)
  {
    calibrationStartTime = system_us;
    calibrationMaxValue = 0;
    calibrationMinValue = 65535;
    calibrationSumValue = 0;
    calibrationCount = 0;
    return;
  }
  if (system_us - calibrationStartTime > 3000000)
  {
    // 3s内没有变化, 自动校准值
    if (trigDirection == 1)
    {
      compareValue = calibrationSumValue / calibrationCount + trigThreshold;
    }
    else
    {
      compareValue = calibrationSumValue / calibrationCount - trigThreshold;
      if (compareValue < 0)
      {
        compareValue = 0;
      }
    }
    if (!btnIsSetting)
    {
      setLed1State(2);
    }

    calibrationStartTime = system_us;
    calibrationMaxValue = 0;
    calibrationMinValue = 65535;
    calibrationSumValue = 0;
    calibrationCount = 0;
  }
}

// 初始化ADC
void setADCInit()
{
  P1M1 |= 0x10;
  P1M0 &= ~0x10; // ADC1_PIN 高阻输入
  P1M1 |= 0x20;
  P1M0 &= ~0x20; // ADC2_PIN 高阻输入

  DMA_ADC_Config();
  Timer2_Init();
}
//========================================================================
// ---------------------- 运算放大器方案 结束 ------------------------------
//========================================================================

//========================================================================
// ---------------------- 通信指令相关  开始-------------------------
//========================================================================
// 重启, 复位
void rebootChip()
{
  IAP_CONTR = 0x60; // 重启进入ISP下载
}
// 恢复默认波特率
void resetUart1Baud()
{
  uart1Baud = 2000000;
  changeBaudTimeoutId = 0;
  uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, uart1Baud, TIM_1); // 初始化串口
  UART1_enable = 1;
}
// 改变通信波特率
void changeUart1Baud()
{
  // 串口波特率改变
  uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, uart1Baud, TIM_1);
  UART1_enable = 1;
  ES = 1; // 允许串行口中断
  // 10秒后未通信成功,恢复波特率
  changeBaudTimeoutId = setTimeout(resetUart1Baud, 10000);
}
// 发送配置参数
void sendDeviceParams()
{
  uint8 i = 0;
  uint8 *buffer = NULL;     // DMA缓存
  int8 dmaBufferIndex = -1; // DMA缓存的索引
  uint8 tempTH0 = 0;
  uint8 tempTL0 = 0;
  uint32 tempGlobeTime = 0;
  do
  {
    // 如果在读取过程中发生溢出，重新读取
    tempTH0 = TH0;
    tempTL0 = TL0;
    tempGlobeTime = globeTime;
  } while (tempTH0 != TH0 || tempTL0 != TL0 || tempGlobeTime != globeTime);

  dmaBufferIndex = UART1_getIdleGroupIndex();
  if (dmaBufferIndex == -1)
  {
    return;
  }
  buffer = &Uart1_sendDMABuffer[dmaBufferIndex][0];

  buffer[i++] = uartCommand.Start1;
  buffer[i++] = uartCommand.Start2;
  buffer[i++] = 0x02;
  buffer[i++] = 0x01;
  buffer[i++] = version;
  // 比较触发值
  buffer[i++] = trigThreshold >> 8;
  buffer[i++] = trigThreshold & 0x00ff;
  // 采样通道
  buffer[i++] = 0;
  // 校准误差
  buffer[i++] = calibrationError >> 8;
  buffer[i++] = calibrationError & 0x00ff;
  // 触发方向
  buffer[i++] = trigDirection;
  // 输出方向
  buffer[i++] = outputDirection;
  // adcValue
  buffer[i++] = adcValue >> 8;
  buffer[i++] = adcValue & 0x00ff;
  // compareValue
  buffer[i++] = compareValue >> 8;
  buffer[i++] = compareValue & 0x00ff;
  // micros()
  buffer[i++] = tempGlobeTime >> 24;
  buffer[i++] = tempGlobeTime >> 16;
  buffer[i++] = tempGlobeTime >> 8;
  buffer[i++] = tempGlobeTime & 0x00ff;
  buffer[i++] = tempTH0;
  buffer[i++] = tempTL0;
  // loopCount
  buffer[i++] = loopCount >> 24;
  buffer[i++] = loopCount >> 16;
  buffer[i++] = loopCount >> 8;
  buffer[i++] = loopCount & 0x00ff;
  // maxLoopTime
  buffer[i++] = maxLoopTime >> 24;
  buffer[i++] = maxLoopTime >> 16;
  buffer[i++] = maxLoopTime >> 8;
  buffer[i++] = maxLoopTime & 0x00ff;
  // UART1_sendCount
  buffer[i++] = UART1_sendCount >> 24;
  buffer[i++] = UART1_sendCount >> 16;
  buffer[i++] = UART1_sendCount >> 8;
  buffer[i++] = UART1_sendCount & 0x00ff;
  // 测试
  buffer[i++] = debugInt1 >> 24;
  buffer[i++] = debugInt1 >> 16;
  buffer[i++] = debugInt1 >> 8;
  buffer[i++] = debugInt1 & 0x00ff;
  // 测试2
  buffer[i++] = debugInt2 >> 24;
  buffer[i++] = debugInt2 >> 16;
  buffer[i++] = debugInt2 >> 8;
  buffer[i++] = debugInt2 & 0x00ff;
  // adc极性
  buffer[i++] = adcPolarity;

  buffer[i++] = getCheckSum(buffer, 2, i); // 校验和
  buffer[i++] = i - 2;
  buffer[i++] = uartCommand.End1;
  buffer[i++] = uartCommand.End2;
  UART1_writeBufferByDMA(dmaBufferIndex, i);
  maxLoopTime = 0;
}
// 保存配置到EEPROM
void saveConfigToEEPROM()
{
  uint8 i = 0;
  uint8 j = 0;
  cmdData[i++] = uartCommand.Start1; // 标识
  cmdData[i++] = uartCommand.Start2;
  cmdData[i++] = version; // 程序版本号
  cmdData[i++] = trigThreshold >> 8;
  cmdData[i++] = trigThreshold & 0x00ff;
  cmdData[i++] = calibrationError >> 8;
  cmdData[i++] = calibrationError & 0x00ff;
  cmdData[i++] = trigDirection;
  cmdData[i++] = outputDirection;
  for (j = 0; j < 5; j++)
  {
    cmdData[i++] = btnTrigThresholdArray[j] >> 8;
    cmdData[i++] = btnTrigThresholdArray[j] & 0x00ff;
    cmdData[i++] = btnCalibrationErrorArray[j];
  }
  cmdData[i++] = enableActivatePin;
  cmdData[i++] = activateProtectValue >> 8;
  cmdData[i++] = activateProtectValue & 0x00ff;
  eeprom_sector_erase(0);          // EEPROM擦除指定地址所在扇区
  eeprom_write(0, cmdData, i + 2); // EEPROM写数据
}
// 从EEPROM读取配置
void readConfigFromEEPROM()
{
  uint8 i = 0;
  uint8 j = 0;
  eeprom_read(0, cmdData, 32); // EEPROM读数据
  if (cmdData[0] == uartCommand.Start1 && cmdData[1] == uartCommand.Start2)
  {
    // 程序版本号
    if (cmdData[2] == version)
    {
      i = 3;
      trigThreshold = (cmdData[i++] << 8) | cmdData[i++];
      calibrationError = (cmdData[i++] << 8) | cmdData[i++];
      trigDirection = cmdData[i++];
      outputDirection = cmdData[i++];
      for (j = 0; j < 5; j++)
      {
        btnTrigThresholdArray[j] = (cmdData[i++] << 8) | cmdData[i++];
        btnCalibrationErrorArray[j] = cmdData[i++];
      }
      enableActivatePin = cmdData[i++];
      activateProtectValue = (cmdData[i++] << 8) | cmdData[i++];
      if (trigDirection > 1)
      {
        trigDirection = 1;
      }
      if (outputDirection > 1)
      {
        outputDirection = 1;
      }
      if (enableActivatePin > 1)
      {
        enableActivatePin = 1;
      }
    }
  }
}
// 上位机设置参数
void setConfigParams(uint8 *buffer, uint8 length)
{
  uint16 lastTrigThreshold = trigThreshold; // 触发阈值
  uint8 i = 2;
  if (buffer[i++] != version)
  {
    return;
  }
  trigThreshold = (buffer[i++] << 8) | buffer[i++];
  sampleChannel = buffer[i++];
  calibrationError = (buffer[i++] << 8) | buffer[i++];
  trigDirection = buffer[i++];
  outputDirection = buffer[i++];
  if (trigDirection > 1)
  {
    trigDirection = 1;
  }
  if (outputDirection > 1)
  {
    outputDirection = 1;
  }

  // 更新比较触发值
  if (lastTrigThreshold > trigThreshold)
  {
    compareValue = compareValue - (lastTrigThreshold - trigThreshold);
  }
  else
  {
    compareValue = compareValue + (trigThreshold - lastTrigThreshold);
  }

  // 避免频繁操作EEPROM, 延时5s保存到EEPROM
  clearTime(EEPROMSaveTimeoutId);
  EEPROMSaveTimeoutId = setTimeout(saveConfigToEEPROM, 5000);
}
// 发送确认信号
void sendAck()
{
  uint8 i = 0;
  uint8 *buffer = NULL;     // DMA缓存
  int8 dmaBufferIndex = -1; // DMA缓存的索引
  dmaBufferIndex = UART1_getIdleGroupIndex();
  if (dmaBufferIndex == -1)
  {
    return;
  }
  buffer = &Uart1_sendDMABuffer[dmaBufferIndex][0];
  buffer[i++] = uartCommand.Start1;
  buffer[i++] = uartCommand.Start2;
  buffer[i++] = 0x01;
  buffer[i++] = 0x01;
  buffer[i++] = 0x00;
  buffer[i++] = 0x00;
  buffer[i++] = getCheckSum(buffer, 2, i); // 校验和
  buffer[i++] = i - 2;
  buffer[i++] = uartCommand.End1;
  buffer[i++] = uartCommand.End2;
  UART1_writeBufferByDMA(dmaBufferIndex, i);
}
// 超过一定时间未接收到信号就自动停止发送
void autoSampleADCTimeoutCallback()
{
  sampleTime = 0;
  sampleTimeCount = 0;
}
// 上位机设置参数2
void setConfigParams2(uint8 *buffer, uint8 length)
{
  uint8 i = 2;
  uint8 j = 0;
  // 设置按钮挡位
  for (j = 0; j < 5; j++)
  {
    btnTrigThresholdArray[j] = buffer[i++];
    btnTrigThresholdArray[j] <<= 8;
    btnTrigThresholdArray[j] |= buffer[i++];
    btnCalibrationErrorArray[j] = buffer[i++];
  }
  // 设置是否启用激活probe引脚
  enableActivatePin = buffer[i++];
  // 设置极限保护值
  activateProtectValue = buffer[i++];
  activateProtectValue <<= 8;
  activateProtectValue |= buffer[i++];
  if (enableActivatePin > 1)
  {
    enableActivatePin = 1;
  }
  // 避免频繁操作EEPROM, 延时5s保存到EEPROM
  clearTime(EEPROMSaveTimeoutId);
  EEPROMSaveTimeoutId = setTimeout(saveConfigToEEPROM, 5000);
}
// 发送配置参数2
void sendConfigParams2()
{
  uint8 i = 0;
  uint8 j = 0;
  uint8 *buffer = NULL;     // DMA缓存
  int8 dmaBufferIndex = -1; // DMA缓存的索引
  dmaBufferIndex = UART1_getIdleGroupIndex();
  if (dmaBufferIndex == -1)
  {
    return;
  }
  buffer = &Uart1_sendDMABuffer[dmaBufferIndex][0];
  buffer[i++] = uartCommand.Start1;
  buffer[i++] = uartCommand.Start2;
  buffer[i++] = 0x04;
  buffer[i++] = 0x02;
  buffer[i++] = 5;
  for (j = 0; j < 5; j++)
  {
    buffer[i++] = btnTrigThresholdArray[j] >> 8;
    buffer[i++] = btnTrigThresholdArray[j] & 0x00ff;
    buffer[i++] = btnCalibrationErrorArray[j];
  }
  buffer[i++] = enableActivatePin;
  buffer[i++] = activateProtectValue >> 8;
  buffer[i++] = activateProtectValue & 0x00ff;
  buffer[i++] = getCheckSum(buffer, 2, i); // 校验和
  buffer[i++] = i - 2;
  buffer[i++] = uartCommand.End1;
  buffer[i++] = uartCommand.End2;
  UART1_writeBufferByDMA(dmaBufferIndex, i);
}
// 接收指令回调
void receiveUartDataCallback(uint8 *buffer, uint8 length)
{
  // 重启芯片, 使用stc isp下载程序
  if (buffer[0] == 0x06 && buffer[1] == 0x07)
  {
    sendAck();
    // 10s后重启
    setTimeout(rebootChip, 10000);
  }
  // 动态改变波特率
  else if (buffer[0] == 0x08 && buffer[1] == 0x09)
  {
    // 关闭自动发送ADC值
    clearTime(autoSampleADCTimeoutId);
    sampleTime = 0;
    sampleTimeCount = 0;
    // 串口波特率改变
    uart1Baud = ((uint32)buffer[2] << 24) | ((uint32)buffer[3] << 16) | ((uint32)buffer[4] << 8) | buffer[5];
    UART1_enable = 0; // 关闭串口
    TR1 = 0;
    ES = 0; // 允许串行口中断
    // 500ms后改变波特率
    setTimeout(changeUart1Baud, 500);
  }
  // 自动发送ADC值
  // f0 f1 01 02 00 00 05 e0 e1
  else if (buffer[0] == 0x01 && buffer[1] == 0x02)
  {
    // 自动发送状态的时间间隔
    sampleTime = (buffer[2] << 8) | buffer[3]; // 采样时间us
    sampleTimeCount = sampleTime / 5;          // adc采样计数，每一次5us
    clearTime(autoSampleADCTimeoutId);
    if (sampleTime > 0)
    {
      autoSampleADCTimeoutId = setTimeout(autoSampleADCTimeoutCallback, 5000); // 超过一定时间未接收到信号就自动停止发送
    }
    setTimeout_us(sendDeviceParams, 1000); // 报告当前状态
  }
  // 读取配置参数
  // f0 f1 02 02 03 e0 e1
  else if (buffer[0] == 0x02 && buffer[1] == 0x02)
  {
    setTimeout_us(sendDeviceParams, 1000); // 报告当前状态
    if (changeBaudTimeoutId > 0)
    {
      clearTime(changeBaudTimeoutId);
    }
  }
  // 设置配置参数, 参数数值就是 读取配置参数的参数
  else if (buffer[0] == 0x03 && buffer[1] == 0x02)
  {
    setConfigParams(buffer, length);
  }
  // 设置配置参数2
  else if (buffer[0] == 0x04 && buffer[1] == 0x01)
  {
    setConfigParams2(buffer, length);
    setTimeout_us(sendConfigParams2, 5000);
  }
  // 发送配置参数2
  else if (buffer[0] == 0x04 && buffer[1] == 0x02)
  {
    setTimeout_us(sendConfigParams2, 1000);
  }
}
//========================================================================
// ---------------------- 通信指令相关  结束-------------------------
//========================================================================

//========================================================================
// ---------------------- LED, 按钮  开始-------------------------
//========================================================================
uint32 led1TimeId = 0;           // led1的定时器id
uint8 led1State2ChangeCount = 0; // led1State2状态改变的次数

struct MyBtn changeBtn;
uint8 btnClickIndex = 0;         // 按钮点击次数
uint8 led1State4ChangeCount = 0; // led1State4状态改变的次数
uint32 btnSettingTimeoutId = 0;  // 按钮设置超时定时器id

void led1State1Callback()
{
  LED_PIN = !LED_PIN;
}
void led1State2Callback()
{
  LED_PIN = !LED_PIN;
  led1State2ChangeCount++;
  if (led1State2ChangeCount > 16)
  {
    // 恢复正常状态
    setLed1State(1);
  }
}
void led1State4Callback()
{
  LED_PIN = !LED_PIN;
  led1State4ChangeCount++;
  if (led1State4ChangeCount == btnClickIndex << 1)
  {
    setLed1State(3);
  }
}
void setLed1State(uint8 led1State)
{
  clearTime(led1TimeId);
  // 正常工作, 每0.5s闪烁一次
  if (led1State == 1)
  {
    led1TimeId = setInterval(led1State1Callback, 500);
  }
  // 自动校准成功, 快闪几次
  if (led1State == 2)
  {
    led1State2ChangeCount = 0;
    led1TimeId = setInterval(led1State2Callback, 30);
  }
  // 按钮设置状态时, 保持熄灭
  if (led1State == 3)
  {
    led1State4ChangeCount = 0;
    LED_PIN = 0;
  }
  // 按钮设置状态时, 闪几次就表示几档
  if (led1State == 4)
  {
    led1State4ChangeCount = 0;
    LED_PIN = 0;
    led1TimeId = setInterval(led1State4Callback, 200);
  }
}
void changeSettingTimeoutCallback()
{
  // 设置超时, 退出设置状态
  btnIsSetting = 0;
  setLed1State(1);
}
// 触发值 按钮档位调节
void changeBtnClick()
{
  if (!btnIsSetting)
  {
    return;
  }
  if (btnClickIndex > 4)
  {
    btnClickIndex = 0;
  }
  trigThreshold = btnTrigThresholdArray[btnClickIndex];
  calibrationError = btnCalibrationErrorArray[btnClickIndex];
  btnClickIndex++;

  setLed1State(4);

  // 避免频繁操作EEPROM, 延时5s保存到EEPROM
  clearTime(EEPROMSaveTimeoutId);
  EEPROMSaveTimeoutId = setTimeout(saveConfigToEEPROM, 5000);

  clearTime(btnSettingTimeoutId);
  btnSettingTimeoutId = setTimeout(changeSettingTimeoutCallback, 10000); // 10s超时
}
// 长按按钮, 进入设置状态
void changeBtnLongPress()
{
  btnIsSetting = !btnIsSetting;
  if (btnIsSetting)
  {
    // 开始设置
    btnClickIndex = 0;
    setLed1State(3);
    clearTime(btnSettingTimeoutId);
    btnSettingTimeoutId = setTimeout(changeSettingTimeoutCallback, 10000); // 10s超时
  }
  else
  {
    clearTime(btnSettingTimeoutId);
    // 结束设置
    setLed1State(1);
  }
}
void refreshBtn()
{
  // 按钮状态检测
  if (BTN_PIN == 0)
  {
    MyBtn_check(&changeBtn, 1);
  }
  else
  {
    MyBtn_check(&changeBtn, 0);
  }
}
void initLedBtn()
{
  P3M1 &= ~0x20;
  P3M0 |= 0x20; // 推挽输出
  P3M1 &= ~0x10;
  P3M0 |= 0x10; // 推挽输出
  P1M1 &= ~0x02;
  P1M0 |= 0x02; // 推挽输出
  P3M1 &= ~0x80;
  P3M0 |= 0x80; // 推挽输出

  P1M0 &= ~0x01;
  P1M1 |= 0x01; // 按钮 高祖输入
  P1PU |= 0x01; // 按钮上拉4k电阻

  PROBE_OUT_PIN = 0;
  PROBE_OUT2_PIN = 0;
  LED_PIN = 0;
  LED2_PIN = 0;

  // 按钮初始化
  MyBtn_init(&changeBtn);
  changeBtn.keydownCallback = NULL;
  changeBtn.keyupCallback = changeBtnClick;
  changeBtn.longPressCallback = changeBtnLongPress;
  changeBtn.doubleClickCallback = NULL;
  setInterval(refreshBtn, 1); // 1ms检测一次按钮状态
  setLed1State(1);            // 设置LED正常工作状态
}
//========================================================================
// ---------------------- LED, 按钮  结束-------------------------
//========================================================================

void setup()
{
  rcclock_set_irc(1);
  WTST = 0;  // 设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
  CKCON = 0; // 提高访问XRAM速度

  uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, uart1Baud, TIM_1); // 初始化串口
  JSTime_init();
  UART1_initCommand(receiveUartDataCallback);
  EA = 1; // 允许总中断
  readConfigFromEEPROM();
  setADCInit();
  initLedBtn();
  setInterval_us(autoInitADC, 100); // 自动校准, 100us执行一次
  WDT_CONTR = 0x23;                 // 使能看门狗
}

void main(void)
{
  setup();
  lastLoopTime = micros();
  while (1)
  {
    micros();
    // 记录任务最大循环时间
    if (system_us - lastLoopTime > maxLoopTime)
    {
      maxLoopTime = system_us - lastLoopTime;
    }
    lastLoopTime = system_us;
    // uart1接收数据
    while (uartReceiveFIFO.headPos != uartReceiveFIFO.tailPos)
    {
      MyCommand_addData(&uartCommand, FIFOBuffer_read(&uartReceiveFIFO));
    }
    // 刷新定时器, 执行定时任务
    JSTime_refresh();
    // 发送数据
    UART1_sendData();
    // 发送ADC数据, 20个数据一起发送
    if (adcCurrentIndex >= 20)
    {
      sendADCSampleSimple();
      adcCurrentIndex = 0; // 重置
    }
    WDT_CONTR = 0x33; // 清看门狗,否则系统复位
    loopCount++;
  }
}
