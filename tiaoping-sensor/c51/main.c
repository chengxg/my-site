// 该程序使用天问block开发工具编译
// 选用的芯片为stc32g8k64

#define IRC_24M
#define PLL_NO
#define boolean unsigned char
#define true 1
#define false 0
#define HIGH 1
#define LOW 0

#include <STC32G.h>
uint32 sys_clk = 24000000; // 设置PWM、定时器、串口、EEPROM频率参数
#include <stdio.h>
#include "lib/rcclock.h"
#include "lib/UART.h"
#include "lib/ADC.h"
#include "lib/delay.h"
#include "lib/eeprom.h"
#include "myLib/MyCommand.h"
#include "myLib/JSTime.h"

#define ADC1_PIN ADC_P14    // ADC通道1
#define ADC2_PIN ADC_P10    // ADC通道2
#define ADC3_PIN ADC_P15    // ADC通道3
#define ADC4_PIN ADC_P11    // ADC通道4
#define PROBE_OUT_PIN P2_2  // 信号输出
#define PROBE_OUT2_PIN P2_3 // 信号输出 反相
#define LED_PIN P2_0        // 状态指示灯
#define LED2_PIN P2_1       // 信号输出指示灯

uint8 version = 0x01; // 程序版本号
uint32 loopCount = 0; // 程序循环次数

uint16 adc1Init = 0; // adc通道初始值
uint16 adc2Init = 0;
uint16 adc3Init = 0;
uint16 adc4Init = 0;
uint16 adc1Base = 32768; // adc通道偏移值32768就是0, 上位机显示时会减去32768
uint16 adc2Base = 32768;
uint16 adc3Base = 32768;
uint16 adc4Base = 32768;
uint32 compareValue = 0; // 比较值 adcxInit * adcxRate + adcxBase

uint8 adc1Rate = 10; // adc通道的放大倍率, 10就是基准,上位机显示时会除以10 与传感器的放大倍数有关, 需要校准
uint8 adc2Rate = 10;
uint8 adc3Rate = 10;
uint8 adc4Rate = 10;

uint16 adc1 = 0;
uint16 adc2 = 0;
uint16 adc3 = 0;
uint16 adc4 = 0;
uint32 currentValue = 0;         // 四个通道的ADC*10 数值扩大了10倍
int32 tempDiffValue = 0;         // 当前值与初始值的差 currentValue - compareValue
uint16 trigThresholdHigh = 3400; // 信号输出触发阈值 高 数值扩大了10倍
uint16 trigThresholdLow = 3000;  // 信号输出触发阈值 低 数值扩大了10倍

#define ADCSampleSize 8 // adc采样数量
uint16 adc1Arr[ADCSampleSize] = {0};
uint16 adc2Arr[ADCSampleSize] = {0};
uint16 adc3Arr[ADCSampleSize] = {0};
uint16 adc4Arr[ADCSampleSize] = {0};

// 热床静止自动校准
#define CalibrationSize 30
uint8 addCalibrationIndex = 0;
uint16 calibrationError = 800; // 检测到这些CalibrationSize数两两之间的差, 小于这个就自动校准
uint32 calibrationArr[CalibrationSize] = {0};

uint32 led1TimeId = 0;                  // led1的定时器id
uint8 led1State2ChangeCount = 0;        // led1State2状态改变的次数
uint16 autoSendStateTime = 0;           // 自动发送状态的时间间隔, ms
uint32 autoSendStateTimeIntervalId = 0; // 自动发送状态的定时器id
uint32 autoSendStateTimeoutId = 0;      // 自动发送状态的定时器超时id, 超过一定时间未接收到信号就自动停止发送
uint32 lastAutoSendStateTime = 0;       // 上一次 自动发送状态的时间间隔, 用于比较改变
uint32 EEPROMSaveTimeoutId = 0;         // EEPROM保存配置的定时器id

uint16 average_without_min_max(uint16 *arr, int length);
uint16 average_arr(uint16 *arr, int length);
void initADC();
void getADC();
void getADCAverage(int size);
void bedOutputSign();
void setLed1State(uint8 led1State);
void sendProgramStatus();

// 编写一个方法计算字节的校验和
uint8 getCheckSum(uint8 *buffer, uint8 start, uint8 end)
{
  uint8 i = 0;
  uint8 sum = 0;
  for (i = start; i < end; i++)
  {
    sum += buffer[i];
  }
  return sum;
}
void sendConfigParams()
{
  uint8 i = 0;
  uint8 cmdData[50] = {0};
  uint32 curTime = micros();
  cmdData[i++] = uartCommand.Start1;
  cmdData[i++] = uartCommand.Start2;
  cmdData[i++] = 0x02;
  cmdData[i++] = 0x01;
  cmdData[i++] = version;
  // adc1Rate, adc2Rate, adc3Rate, adc4Rate
  cmdData[i++] = adc1Rate;
  cmdData[i++] = adc2Rate;
  cmdData[i++] = adc3Rate;
  cmdData[i++] = adc4Rate;
  // adc1Base, adc2Base, adc3Base, adc4Base
  cmdData[i++] = adc1Base >> 8;
  cmdData[i++] = adc1Base & 0x00ff;
  cmdData[i++] = adc2Base >> 8;
  cmdData[i++] = adc2Base & 0x00ff;
  cmdData[i++] = adc3Base >> 8;
  cmdData[i++] = adc3Base & 0x00ff;
  cmdData[i++] = adc4Base >> 8;
  cmdData[i++] = adc4Base & 0x00ff;
  // adc1Init, adc2Init, adc3Init, adc4Init
  cmdData[i++] = adc1Init >> 8;
  cmdData[i++] = adc1Init & 0x00ff;
  cmdData[i++] = adc2Init >> 8;
  cmdData[i++] = adc2Init & 0x00ff;
  cmdData[i++] = adc3Init >> 8;
  cmdData[i++] = adc3Init & 0x00ff;
  cmdData[i++] = adc4Init >> 8;
  cmdData[i++] = adc4Init & 0x00ff;
  // trigThresholdHigh, trigThresholdLow, calibrationError
  cmdData[i++] = trigThresholdHigh >> 8;
  cmdData[i++] = trigThresholdHigh & 0x00ff;
  cmdData[i++] = trigThresholdLow >> 8;
  cmdData[i++] = trigThresholdLow & 0x00ff;
  cmdData[i++] = calibrationError >> 8;
  cmdData[i++] = calibrationError & 0x00ff;
  // micros()
  cmdData[i++] = curTime >> 24;
  cmdData[i++] = curTime >> 16;
  cmdData[i++] = curTime >> 8;
  cmdData[i++] = curTime & 0x000000ff;
  // loopCount
  cmdData[i++] = loopCount >> 24;
  cmdData[i++] = loopCount >> 16;
  cmdData[i++] = loopCount >> 8;
  cmdData[i++] = loopCount & 0x000000ff;

  cmdData[i++] = getCheckSum(cmdData, 2, i); // 校验和
  cmdData[i++] = i - 2;
  cmdData[i++] = uartCommand.End1;
  cmdData[i++] = uartCommand.End2;

  uart1WriteBuf(cmdData, i);
}
// 保存配置到EEPROM
void saveConfigToEEPROM()
{
  uint8 i = 0;
  uint8 buffer[32] = {0};
  buffer[i++] = 0xf0; // 标识
  buffer[i++] = 0xf1;
  buffer[i++] = version; // 程序版本号
  buffer[i++] = adc1Rate;
  buffer[i++] = adc2Rate;
  buffer[i++] = adc3Rate;
  buffer[i++] = adc4Rate;
  buffer[i++] = adc1Base >> 8;
  buffer[i++] = adc1Base & 0x00ff;
  buffer[i++] = adc2Base >> 8;
  buffer[i++] = adc2Base & 0x00ff;
  buffer[i++] = adc3Base >> 8;
  buffer[i++] = adc3Base & 0x00ff;
  buffer[i++] = adc4Base >> 8;
  buffer[i++] = adc4Base & 0x00ff;
  buffer[i++] = trigThresholdHigh >> 8;
  buffer[i++] = trigThresholdHigh & 0x00ff;
  buffer[i++] = trigThresholdLow >> 8;
  buffer[i++] = trigThresholdLow & 0x00ff;
  buffer[i++] = calibrationError >> 8;
  buffer[i++] = calibrationError & 0x00ff;

  eeprom_sector_erase(0);      // EEPROM擦除指定地址所在扇区
  eeprom_write(0, buffer, 32); // EEPROM写数据
}
// 从EEPROM读取配置
void readConfigFromEEPROM()
{
  uint8 i = 0;
  uint8 buffer[32] = {0};
  eeprom_read(0, buffer, 32); // EEPROM读数据
  uart1WriteBuf(buffer, 32);
  if (buffer[0] == 0xf0 && buffer[1] == 0xf1)
  {
    // 程序版本号
    if (buffer[2] == version)
    {
      i = 3;
      adc1Rate = buffer[i++];
      adc2Rate = buffer[i++];
      adc3Rate = buffer[i++];
      adc4Rate = buffer[i++];
      adc1Base = (buffer[i++] << 8) | buffer[i++];
      adc2Base = (buffer[i++] << 8) | buffer[i++];
      adc3Base = (buffer[i++] << 8) | buffer[i++];
      adc4Base = (buffer[i++] << 8) | buffer[i++];
      trigThresholdHigh = (buffer[i++] << 8) | buffer[i++];
      trigThresholdLow = (buffer[i++] << 8) | buffer[i++];
      calibrationError = (buffer[i++] << 8) | buffer[i++];
    }
  }
}
// 上位机设置参数
void setConfigParams(uint8 *buffer, uint8 length)
{
  uint8 i = 2;
  if (length < 10)
  {
    return;
  }
  adc1Rate = buffer[i++];
  adc2Rate = buffer[i++];
  adc3Rate = buffer[i++];
  adc4Rate = buffer[i++];
  adc1Base = (buffer[i++] << 8) | buffer[i++];
  adc2Base = (buffer[i++] << 8) | buffer[i++];
  adc3Base = (buffer[i++] << 8) | buffer[i++];
  adc4Base = (buffer[i++] << 8) | buffer[i++];
  trigThresholdHigh = (buffer[i++] << 8) | buffer[i++];
  trigThresholdLow = (buffer[i++] << 8) | buffer[i++];
  calibrationError = (buffer[i++] << 8) | buffer[i++];

  // 避免频繁操作EEPROM, 延时5s保存到EEPROM
  clearTime(EEPROMSaveTimeoutId);
  EEPROMSaveTimeoutId = setTimeout(saveConfigToEEPROM, 5000);
}
void sendProgramStatus()
{
  uint8 i = 0;
  uint32 curTime = micros();
  uint8 cmdData[20] = {0};
  cmdData[i++] = uartCommand.Start1;
  cmdData[i++] = uartCommand.Start2;
  cmdData[i++] = 0x01;
  cmdData[i++] = 0x01;
  // adc1, adc2, adc3, adc4
  cmdData[i++] = adc1 >> 8;
  cmdData[i++] = adc1 & 0x00ff;
  cmdData[i++] = adc2 >> 8;
  cmdData[i++] = adc2 & 0x00ff;
  cmdData[i++] = adc3 >> 8;
  cmdData[i++] = adc3 & 0x00ff;
  cmdData[i++] = adc4 >> 8;
  cmdData[i++] = adc4 & 0x00ff;

  cmdData[i++] = getCheckSum(cmdData, 2, i); // 校验和
  cmdData[i++] = i - 2;                      // 数据长度
  cmdData[i++] = uartCommand.End1;
  cmdData[i++] = uartCommand.End2;

  uart1WriteBuf(cmdData, i);
}
void autoSendStateCallback()
{
  if (autoSendStateTime == 0)
  {
    clearTime(autoSendStateTimeIntervalId);
    return;
  }
  sendProgramStatus();
}
void autoSendStateTimeoutCallback()
{
  clearTime(autoSendStateTimeIntervalId);
  autoSendStateTimeIntervalId = 0;
  autoSendStateTimeoutId = 0;
  autoSendStateTime = 0;
  lastAutoSendStateTime = 0;
}
void receiveUartDataCallback(uint8 *buffer, uint8 length)
{
  // 读取ADC值
  // f0 f1 01 02 00 00 05 e0 e1
  if (buffer[0] == 0x01 && buffer[1] == 0x02)
  {
    sendProgramStatus();
    // 自动发送状态的时间间隔
    autoSendStateTime = (buffer[2] << 8) | buffer[3];
    if (autoSendStateTime > 0)
    {
      if (lastAutoSendStateTime != autoSendStateTime)
      {
        clearTime(autoSendStateTimeIntervalId);
        autoSendStateTimeIntervalId = setInterval(autoSendStateCallback, autoSendStateTime / 4.0);
        lastAutoSendStateTime = autoSendStateTime;
      }
      // 超过一定时间未接收到信号就自动停止发送
      clearTime(autoSendStateTimeoutId);
      autoSendStateTimeoutId = setTimeout(autoSendStateTimeoutCallback, 5000);
    }
    else
    {
      // autoSendStateTime=0 停止自动发送状态
      if (autoSendStateTimeIntervalId > 0)
      {
        clearTime(autoSendStateTimeIntervalId);
        autoSendStateTimeIntervalId = 0;
        lastAutoSendStateTime = 0;
      }
    }
  }
  // 读取配置参数
  // f0 f1 02 02 03 e0 e1
  else if (buffer[0] == 0x02 && buffer[1] == 0x02)
  {
    sendConfigParams();
  }
  // 设置配置参数, 参数数值就是 读取配置参数的参数
  else if (buffer[0] == 0x03 && buffer[1] == 0x02)
  {
    setConfigParams(buffer, length);
  }
}

uint16 my_adc_read(ADC_Name adcn)
{
  // adc取一次丢掉, 再获取一次
  adc_read(adcn);
  return adc_read(adcn);
}

uint16 average_without_min_max(uint16 *arr, uint8 length)
{
  int sum = 0;
  uint16 min = 65535;
  uint16 max = 0;
  uint8 i = 0;
  if (length <= 2)
  {
    return 0; // 如果数组长度小于等于2，返回0
  }

  for (i = 0; i < length; i++)
  {
    if (arr[i] < min)
    {
      min = arr[i];
    }
    if (arr[i] > max)
    {
      max = arr[i];
    }
    sum += arr[i];
  }

  sum -= min + max;          // 去掉最大值和最小值
  return sum / (length - 2); // 计算平均值
}

uint16 average_arr(uint16 *arr, uint8 length)
{
  int sum = 0;
  uint8 i = 0;
  for (i = 0; i < length; i++)
  {
    sum += arr[i];
  }
  return sum / length; // 计算平均值
}

void initADC()
{
  uint8 i = 0;
  for (i = 0; i < ADCSampleSize; i++)
  {
    if (adc1Rate != 0)
    {
      adc1Arr[i] = my_adc_read(ADC1_PIN);
    }
    if (adc2Rate != 0)
    {
      adc2Arr[i] = my_adc_read(ADC2_PIN);
    }
    if (adc3Rate != 0)
    {
      adc3Arr[i] = my_adc_read(ADC3_PIN);
    }
    if (adc4Rate != 0)
    {
      adc4Arr[i] = my_adc_read(ADC4_PIN);
    }
  }
  if (adc1Rate != 0)
  {
    adc1Init = average_without_min_max(adc1Arr, ADCSampleSize);
  }
  if (adc2Rate != 0)
  {
    adc2Init = average_without_min_max(adc2Arr, ADCSampleSize);
  }
  if (adc3Rate != 0)
  {
    adc3Init = average_without_min_max(adc3Arr, ADCSampleSize);
  }
  if (adc4Rate != 0)
  {
    adc4Init = average_without_min_max(adc4Arr, ADCSampleSize);
  }
  compareValue = adc1Init * adc1Rate + adc2Init * adc2Rate + adc3Init * adc3Rate +
                 adc4Init * adc4Rate;
}

void getADC()
{
  if (adc1Rate != 0)
  {
    adc1 = my_adc_read(ADC1_PIN);
  }
  if (adc2Rate != 0)
  {
    adc2 = my_adc_read(ADC2_PIN);
  }
  if (adc3Rate != 0)
  {
    adc3 = my_adc_read(ADC3_PIN);
  }
  if (adc4Rate != 0)
  {
    adc4 = my_adc_read(ADC4_PIN);
  }
  currentValue =
      adc1 * adc1Rate + adc2 * adc2Rate + adc3 * adc3Rate + adc4 * adc4Rate;
}

void getADCAverage()
{
  uint8 i = 0;
  for (i = 0; i < ADCSampleSize; i++)
  {
    if (adc1Rate != 0)
    {
      adc1Arr[i] = my_adc_read(ADC1_PIN);
    }
    if (adc2Rate != 0)
    {
      adc2Arr[i] = my_adc_read(ADC2_PIN);
    }
    if (adc3Rate != 0)
    {
      adc3Arr[i] = my_adc_read(ADC3_PIN);
    }
    if (adc4Rate != 0)
    {
      adc4Arr[i] = my_adc_read(ADC4_PIN);
    }
  }
  if (adc1Rate != 0)
  {
    adc1 = average_without_min_max(adc1Arr, ADCSampleSize);
  }
  if (adc2Rate != 0)
  {
    adc2 = average_without_min_max(adc2Arr, ADCSampleSize);
  }
  if (adc3Rate != 0)
  {
    adc3 = average_without_min_max(adc3Arr, ADCSampleSize);
  }
  if (adc4Rate != 0)
  {
    adc4 = average_without_min_max(adc4Arr, ADCSampleSize);
  }
  currentValue =
      adc1 * adc1Rate + adc2 * adc2Rate + adc3 * adc3Rate + adc4 * adc4Rate;
}

void autoInitADC()
{
  // 判断这 CalibrationSize个数, 两两之间的最大的数值差
  int32 maxDiff = 0;
  int32 diff = 0;
  uint8 i = 0;
  uint8 j = 0;
  for (i = 0; i < CalibrationSize; i++)
  {
    for (j = 0; j < CalibrationSize; j++)
    {
      diff = calibrationArr[i] - calibrationArr[j];
      if (diff < 0)
      {
        diff = -diff;
      }
      if (diff > maxDiff)
      {
        maxDiff = diff;
      }
    }
  }
  if (maxDiff <= calibrationError)
  {
    adc1Init = adc1;
    adc2Init = adc2;
    adc3Init = adc3;
    adc4Init = adc4;
    compareValue = currentValue;
    setLed1State(2);
  }
}

// 自动校准初始值, 每200ms取一次数据, 取CalibrationSize个, 然后两两对比
void taskAutoCalibration()
{
  calibrationArr[addCalibrationIndex] = currentValue;
  addCalibrationIndex++;
  if (addCalibrationIndex >= CalibrationSize)
  {
    addCalibrationIndex = 0;
    autoInitADC();
  }
}

// 热床信号触发
void bedOutputSign()
{
  getADC();
  tempDiffValue = currentValue - compareValue;
  if (tempDiffValue >= trigThresholdHigh)
  {
    getADCAverage();
    tempDiffValue = currentValue - compareValue;
    if (tempDiffValue >= trigThresholdHigh)
    {
      PROBE_OUT_PIN = HIGH;
      PROBE_OUT2_PIN = LOW;
      LED2_PIN = HIGH;
    }
  }
  else if (tempDiffValue < trigThresholdLow)
  {
    PROBE_OUT_PIN = LOW;
    PROBE_OUT2_PIN = HIGH;
    LED2_PIN = LOW;
  }
}

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
}

void mytask1()
{
}

void setup()
{
  rcclock_set_irc(1);
  P2M1 &= ~0x01;
  P2M0 |= 0x01; // 推挽输出
  P2M1 &= ~0x02;
  P2M0 |= 0x02; // 推挽输出
  P2M1 &= ~0x04;
  P2M0 |= 0x04; // 推挽输出
  P2M1 &= ~0x08;
  P2M0 |= 0x08; // 推挽输出
  P3M1 |= 0x04;
  P3M0 &= ~0x04;                                                 // P3.2 高阻输入
  uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 1000000, TIM_1); // 初始化串口
  adc_init(ADC1_PIN, ADC_SYSclk_DIV_2, ADC_12BIT);
  adc_init(ADC2_PIN, ADC_SYSclk_DIV_2, ADC_12BIT);
  adc_init(ADC3_PIN, ADC_SYSclk_DIV_2, ADC_12BIT);
  adc_init(ADC4_PIN, ADC_SYSclk_DIV_2, ADC_12BIT);
  JSTime_init();
  initUartCommand(receiveUartDataCallback);
  ES = 1; // 允许串行口中断
  EA = 1; // 允许总中断
  delay(500);
  readConfigFromEEPROM();
  initADC();
  setLed1State(1);
  setInterval(taskAutoCalibration, 200); // 自动校准
}

void ispDowbload()
{
  if (P3_2 == LOW)
  {
    if (P3_2 == LOW)
    {
      IAP_CONTR = 0x60; // 进入ISP下载
    }
  }
}

void uartLoopRead()
{
  while (uartReceiveFIFO.headPos != uartReceiveFIFO.tailPos)
  {
    MyCommand_addData(&uartCommand, FIFOBuffer_read(&uartReceiveFIFO));
  }
}

void loop()
{
  JSTime_refresh();
  bedOutputSign();
  ispDowbload();
  uartLoopRead();
  loopCount++;
}

void main(void)
{
  setup();
  while (1)
  {
    loop();
  }
}
