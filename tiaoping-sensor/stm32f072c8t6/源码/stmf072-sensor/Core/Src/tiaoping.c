#include "main.h"
#include "JSTime.h"
#include "MyCommand.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define boolean unsigned char
#define true 1
#define false 0
#define HIGH 1
#define LOW 0

#define FLASH_USER_START_ADDR 0x0800F000 /* Start @ of user Flash area */

// 引脚定义
#define ADC1_PIN ADC_CHANNEL_4    // ADC通道1
#define ADC2_PIN ADC_CHANNEL_2    // ADC通道2
#define ADC3_PIN ADC_CHANNEL_3    // ADC通道3
#define ADC4_PIN ADC_CHANNEL_5    // ADC通道4
#define PROBE_OUT_PIN GPIO_PIN_6  // 信号输出
#define PROBE_OUT2_PIN GPIO_PIN_7 // 信号输出 反相
#define LED_PIN GPIO_PIN_0        // 状态指示灯
#define LED2_PIN GPIO_PIN_1       // 信号输出指示灯

uint8_t version = 0x01;    // 程序版本号
uint32_t loopCount = 0;    // 程序循环次数
uint32_t lastLoopTime = 0; // 上次检测时间
uint32_t curLoopTime = 0;  // 当前检测时间
uint32_t maxTempTime = 0;  // 单次最大循环间隔时间
uint32_t maxLoopTime = 0;  // 单次最大循环间隔时间
uint8_t cmdData[50] = {0};
uint16_t cmd16Data[32] = {0};

uint16_t adc1Init = 0; // adc通道初始值
uint16_t adc2Init = 0;
uint16_t adc3Init = 0;
uint16_t adc4Init = 0;
uint16_t adc1Base = 32768; // adc通道偏移值32768就是0, 上位机显示时会减去32768
uint16_t adc2Base = 32768;
uint16_t adc3Base = 32768;
uint16_t adc4Base = 32768;
uint32_t compareValue = 0; // 比较值 adcxInit * adcxRate + adcxBase

uint8_t adc1Rate = 10; // adc通道的放大倍率, 10就是基准,上位机显示时会除以10 与传感器的放大倍数有关, 需要校准
uint8_t adc2Rate = 10;
uint8_t adc3Rate = 10;
uint8_t adc4Rate = 10;

uint16_t adc1 = 0;
uint16_t adc2 = 0;
uint16_t adc3 = 0;
uint16_t adc4 = 0;
uint32_t currentValue = 0;         // 四个通道的ADC*10 数值扩大了10倍
int32_t tempDiffValue = 0;         // 当前值与初始值的差 currentValue - compareValue
uint16_t trigThresholdHigh = 3400; // 信号输出触发阈值 高 数值扩大了10倍
uint16_t trigThresholdLow = 3000;  // 信号输出触发阈值 低 数值扩大了10倍

#define ADCSampleSize 5 // adc采样数量
uint16_t adc1Arr[ADCSampleSize] = {0};
uint16_t adc2Arr[ADCSampleSize] = {0};
uint16_t adc3Arr[ADCSampleSize] = {0};
uint16_t adc4Arr[ADCSampleSize] = {0};

// 热床静止自动校准
#define CalibrationSize 20
uint8_t addCalibrationIndex = 0;
uint16_t calibrationError = 500; // 检测到这些CalibrationSize数两两之间的差, 小于这个就自动校准
uint32_t calibrationArr[CalibrationSize] = {0};

uint32_t led1TimeId = 0;                  // led1的定时器id
uint8_t led1State2ChangeCount = 0;        // led1State2状态改变的次数
uint16_t autoSendStateTime = 0;           // 自动发送状态的时间间隔, ms
uint32_t autoSendStateTimeIntervalId = 0; // 自动发送状态的定时器id
uint32_t autoSendStateTimeoutId = 0;      // 自动发送状态的定时器超时id, 超过一定时间未接收到信号就自动停止发送
uint32_t lastAutoSendStateTime = 0;       // 上一次 自动发送状态的时间间隔, 用于比较改变
uint32_t EEPROMSaveTimeoutId = 0;         // EEPROM保存配置的定时器id

uint16_t average_without_min_max(uint16_t *arr, uint8_t length);
uint16_t average_arr(uint16_t *arr, uint8_t length);
void initADC();
void getADC();
void getADCAverage();
void bedOutputSign();
void setLed1State(uint8_t led1State);
void sendProgramStatus();
void initSensor();
void loopSensor();

void Flash_WriteBuffer16(uint32_t address, uint16_t *buffer, uint32_t length)
{
  HAL_FLASH_Unlock();

  // 擦除 Flash 扇区
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = address;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    // 擦除错误处理
    Error_Handler();
  }

  // 写入缓冲区到 Flash
  uint32_t i = 0;
  for (i = 0; i < length; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i * 2, buffer[i]) != HAL_OK)
    {
      // 写入错误处理
      Error_Handler();
    }
  }

  HAL_FLASH_Lock();
}

void Flash_ReadBuffer16(uint32_t address, uint16_t *buffer, uint32_t length)
{
  uint32_t i = 0;
  for (i = 0; i < length; i++)
  {
    buffer[i] = *(__IO uint16_t *)(address + i * 2);
  }
}

// 编写一个方法计算字节的校验和
uint8_t getCheckSum(uint8_t *buffer, uint8_t start, uint8_t end)
{
  uint8_t i = 0;
  uint8_t sum = 0;
  for (i = start; i < end; i++)
  {
    sum += buffer[i];
  }
  return sum;
}
void sendConfigParams()
{
  uint8_t i = 0;
  uint32_t curTime = micros();
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
  // maxLoopTime
  cmdData[i++] = maxLoopTime >> 8;
  cmdData[i++] = maxLoopTime & 0x00ff;

  uint8_t checksum = getCheckSum(cmdData, 2, i);
  cmdData[i++] = checksum; // 校验和
  uint8_t length = i - 2;
  cmdData[i++] = length;
  cmdData[i++] = uartCommand.End1;
  cmdData[i++] = uartCommand.End2;

  maxLoopTime = 0;

  uart1WriteBuf(cmdData, i);
}

void sendConfigParams2()
{
  uint8_t i = 0;
  uint32_t curTime = micros();
  cmdData[i++] = uartCommand.Start1;
  cmdData[i++] = uartCommand.Start2;
  cmdData[i++] = 0x02;
  cmdData[i++] = 0x03;

  // adc1Init, adc2Init, adc3Init, adc4Init
  cmdData[i++] = adc1Init >> 8;
  cmdData[i++] = adc1Init & 0x00ff;
  cmdData[i++] = adc2Init >> 8;
  cmdData[i++] = adc2Init & 0x00ff;
  cmdData[i++] = adc3Init >> 8;
  cmdData[i++] = adc3Init & 0x00ff;
  cmdData[i++] = adc4Init >> 8;
  cmdData[i++] = adc4Init & 0x00ff;

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
  // maxLoopTime
  cmdData[i++] = maxLoopTime >> 8;
  cmdData[i++] = maxLoopTime & 0x00ff;

  cmdData[i++] = 0;

  uint8_t checksum = getCheckSum(cmdData, 2, i);
  cmdData[i++] = checksum; // 校验和
  uint8_t length = i - 2;
  cmdData[i++] = length;
  cmdData[i++] = uartCommand.End1;
  cmdData[i++] = uartCommand.End2;

  maxLoopTime = 0;
  uart1WriteBuf(cmdData, i);
}

// 保存配置到EEPROM
void saveConfigToEEPROM()
{
  uint16_t i = 0;
  cmd16Data[i++] = 0xf0; // 标识
  cmd16Data[i++] = 0xf1;
  cmd16Data[i++] = version; // 程序版本号

  cmd16Data[i++] = adc1Rate;
  cmd16Data[i++] = adc2Rate;
  cmd16Data[i++] = adc3Rate;
  cmd16Data[i++] = adc4Rate;

  cmd16Data[i++] = adc1Base;
  cmd16Data[i++] = adc2Base;
  cmd16Data[i++] = adc3Base;
  cmd16Data[i++] = adc4Base;

  cmd16Data[i++] = trigThresholdHigh;
  cmd16Data[i++] = trigThresholdLow;
  cmd16Data[i++] = calibrationError;

  Flash_WriteBuffer16(FLASH_USER_START_ADDR, cmd16Data, 32);
}
// 从EEPROM读取配置
void readConfigFromEEPROM()
{
  uint8_t i = 0;
  Flash_ReadBuffer16(FLASH_USER_START_ADDR, cmd16Data, 32);
  if (cmd16Data[0] == 0xf0 && cmd16Data[1] == 0xf1)
  {
    // 程序版本号
    if (cmd16Data[2] == version)
    {
      i = 3;
      adc1Rate = cmd16Data[i++];
      adc2Rate = cmd16Data[i++];
      adc3Rate = cmd16Data[i++];
      adc4Rate = cmd16Data[i++];

      adc1Base = cmd16Data[i++];
      adc2Base = cmd16Data[i++];
      adc3Base = cmd16Data[i++];
      adc4Base = cmd16Data[i++];

      trigThresholdHigh = cmd16Data[i++];
      trigThresholdLow = cmd16Data[i++];
      calibrationError = cmd16Data[i++];
    }
  }
}
// 上位机设置参数
void setConfigParams(uint8_t *buffer, uint8_t length)
{
  uint8_t i = 2;
  if (length < 10)
  {
    return;
  }
  adc1Rate = buffer[i++];
  adc2Rate = buffer[i++];
  adc3Rate = buffer[i++];
  adc4Rate = buffer[i++];

  adc1Base = (buffer[i] << 8) | buffer[i + 1];
  i += 2;
  adc2Base = (buffer[i] << 8) | buffer[i + 1];
  i += 2;
  adc3Base = (buffer[i] << 8) | buffer[i + 1];
  i += 2;
  adc4Base = (buffer[i] << 8) | buffer[i + 1];
  i += 2;

  trigThresholdHigh = (buffer[i] << 8) | buffer[i + 1];
  i += 2;
  trigThresholdLow = (buffer[i] << 8) | buffer[i + 1];
  i += 2;
  calibrationError = (buffer[i] << 8) | buffer[i + 1];
  i += 2;

  // 避免频繁操作EEPROM, 延时2s保存到EEPROM
  clearTime(EEPROMSaveTimeoutId);
  EEPROMSaveTimeoutId = setTimeout(saveConfigToEEPROM, 2000);
}
void sendProgramStatus()
{
  uint8_t i = 0;
  // uint32_t curTime = micros();
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

  uint8_t checksum = getCheckSum(cmdData, 2, i);
  cmdData[i++] = checksum; // 校验和
  uint8_t length = i - 2;
  cmdData[i++] = length;
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
void receiveUartDataCallback(uint8_t *buffer, uint8_t length)
{
  // 自动发送ADC值
  // f0 f1 01 02 00 00 05 e0 e1
  if (buffer[0] == 0x01 && buffer[1] == 0x02)
  {
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

    setTimeout(sendConfigParams, 1);
  }
  // 读取配置参数
  // f0 f1 02 02 03 e0 e1
  else if (buffer[0] == 0x02 && buffer[1] == 0x02)
  {
    setTimeout(sendConfigParams, 1);
  }
  // 设置配置参数, 参数数值就是 读取配置参数的参数
  else if (buffer[0] == 0x03 && buffer[1] == 0x02)
  {
    setConfigParams(buffer, length);
  }
}

uint16_t Read_ADC_Value(uint32_t channel)
{
  uint16_t adcValue = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  /* 配置 ADC 通道 */
  sConfig.Channel = channel;
  sConfig.Rank = 0;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    /* 配置通道错误处理 */
    Error_Handler();
  }

  // 读取到的各个通道ADC值是相同的
  // 找到HAL_ADC.C库文件里面，
  // 第1606行 hadc->Instance->CHSELR |= ADC_CHSELR_CHANNEL(sConfig->Channel);
  // 把或去掉即可
  // 库函数bug, 不能设置多个通道, 这里直接设置寄存器
  ADC1->CHSELR = 1U << channel;

  /* 启动 ADC */
  HAL_ADC_Start(&hadc);

  /* 等待 ADC 转换完成 */
  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    /* 获取 ADC 转换结果 */
    adcValue = HAL_ADC_GetValue(&hadc);
  }

  /* 停止 ADC */
  HAL_ADC_Stop(&hadc);

  return adcValue;
}

uint16_t my_adc_read(uint32_t channel)
{
  if (channel == ADC1_PIN)
  {
    return adcBuffer[2];
  }
  else if (channel == ADC2_PIN)
  {
    return adcBuffer[0];
  }
  else if (channel == ADC3_PIN)
  {
    return adcBuffer[1];
  }
  else if (channel == ADC4_PIN)
  {
    return adcBuffer[3];
  }
  return 0;
  // return Read_ADC_Value(channel);
}

uint16_t average_without_min_max(uint16_t *arr, uint8_t length)
{
  int sum = 0;
  uint16_t min = 65535;
  uint16_t max = 0;
  uint8_t i = 0;
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

uint16_t average_arr(uint16_t *arr, uint8_t length)
{
  int sum = 0;
  uint8_t i = 0;
  for (i = 0; i < length; i++)
  {
    sum += arr[i];
  }
  return sum / length; // 计算平均值
}

void initADC()
{
  uint8_t i = 0;
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
  compareValue = 0;
  if (adc1Rate != 0)
  {
    adc1Init = average_without_min_max(adc1Arr, ADCSampleSize);
    compareValue += adc1Init * adc1Rate;
  }
  if (adc2Rate != 0)
  {
    adc2Init = average_without_min_max(adc2Arr, ADCSampleSize);
    compareValue += adc2Init * adc2Rate;
  }
  if (adc3Rate != 0)
  {
    adc3Init = average_without_min_max(adc3Arr, ADCSampleSize);
    compareValue += adc3Init * adc3Rate;
  }
  if (adc4Rate != 0)
  {
    adc4Init = average_without_min_max(adc4Arr, ADCSampleSize);
    compareValue += adc4Init * adc4Rate;
  }
}

void getADC()
{
  currentValue = 0;
  if (adc1Rate != 0)
  {
    adc1 = my_adc_read(ADC1_PIN);
    currentValue += adc1 * adc1Rate;
  }
  if (adc2Rate != 0)
  {
    adc2 = my_adc_read(ADC2_PIN);
    currentValue += adc2 * adc2Rate;
  }
  if (adc3Rate != 0)
  {
    adc3 = my_adc_read(ADC3_PIN);
    currentValue += adc3 * adc3Rate;
  }
  if (adc4Rate != 0)
  {
    adc4 = my_adc_read(ADC4_PIN);
    currentValue += adc4 * adc4Rate;
  }
}

void getADCAverage()
{
  uint8_t i = 0;
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
  currentValue = 0;
  if (adc1Rate != 0)
  {
    adc1 = average_arr(adc1Arr, ADCSampleSize);
    currentValue += adc1 * adc1Rate;
  }
  if (adc2Rate != 0)
  {
    adc2 = average_arr(adc2Arr, ADCSampleSize);
    currentValue += adc2 * adc2Rate;
  }
  if (adc3Rate != 0)
  {
    adc3 = average_arr(adc3Arr, ADCSampleSize);
    currentValue += adc3 * adc3Rate;
  }
  if (adc4Rate != 0)
  {
    adc4 = average_arr(adc4Arr, ADCSampleSize);
    currentValue += adc4 * adc4Rate;
  }
}

void autoInitADC()
{
  // 判断这 CalibrationSize个数
  int32_t maxVal = 0;
  int32_t minVal = 65536;
  uint8_t i = 0;
  // 找到最大值和最小值
  for (i = 0; i < CalibrationSize; i++)
  {
    if (calibrationArr[i] < minVal)
    {
      minVal = calibrationArr[i];
    }
    if (calibrationArr[i] > maxVal)
    {
      maxVal = calibrationArr[i];
    }
  }
  if (maxVal - minVal <= calibrationError)
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
    HAL_GPIO_WritePin(GPIOA, PROBE_OUT_PIN, HIGH);
    HAL_GPIO_WritePin(GPIOA, PROBE_OUT2_PIN, LOW);
    HAL_GPIO_WritePin(GPIOA, LED2_PIN, HIGH);
  }
  else if (tempDiffValue < trigThresholdLow)
  {
    HAL_GPIO_WritePin(GPIOA, PROBE_OUT_PIN, LOW);
    HAL_GPIO_WritePin(GPIOA, PROBE_OUT2_PIN, HIGH);
    HAL_GPIO_WritePin(GPIOA, LED2_PIN, LOW);
  }
}

void led1State1Callback()
{
  HAL_GPIO_TogglePin(GPIOA, LED_PIN);
}
void led1State2Callback()
{
  HAL_GPIO_TogglePin(GPIOA, LED_PIN);
  led1State2ChangeCount++;
  if (led1State2ChangeCount > 16)
  {
    // 恢复正常状态
    setLed1State(1);
  }
}
void setLed1State(uint8_t led1State)
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

void uartLoopRead()
{
  while (uartReceiveFIFO.headPos != uartReceiveFIFO.tailPos)
  {
    MyCommand_addData(&uartCommand, FIFOBuffer_read(&uartReceiveFIFO));
  }
}

void uartSendCacheData()
{
  if (!CDC_Transmit_is_USBD_BUSY())
  {
    int i = 0;
    while (uartSendFIFO.headPos != uartSendFIFO.tailPos)
    {
      uartSendCacheBuffer[i] = FIFOBuffer_read(&uartSendFIFO);
      i++;
    }
    if (i > 0)
    {
      CDC_Transmit_FS(uartSendCacheBuffer, i);
    }
  }
}

void initSensor()
{
  lastLoopTime = micros();
  readConfigFromEEPROM();
  initADC();
  initUartCommand(receiveUartDataCallback);
  setLed1State(1);
  setInterval(taskAutoCalibration, 200); // 自动校准
}

void loopSensor()
{
  curLoopTime = micros();
  maxTempTime = curLoopTime - lastLoopTime;
  if (maxTempTime > maxLoopTime)
  {
    maxLoopTime = maxTempTime;
  }
  lastLoopTime = curLoopTime;
  bedOutputSign();
  uartLoopRead();
  uartSendCacheData();
  loopCount++;
}