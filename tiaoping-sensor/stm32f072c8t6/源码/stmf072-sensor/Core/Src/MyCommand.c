#include "MyCommand.h"

#define boolean unsigned char
#define true 1
#define false 0
#define HIGH 1
#define LOW 0

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

#define UartCommandBufferSize 64
unsigned char uartCommandBuffer[UartCommandBufferSize] = {0};
struct MyCommand uartCommand;

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

unsigned char uartReceiveBuffer[UartReceiveSize] = {0};
unsigned char uartSendBuffer[UartSendSize] = {0};
unsigned char uartSendCacheBuffer[UartSendSize] = {0};
struct FIFOBuffer uartReceiveFIFO;
struct FIFOBuffer uartSendFIFO;
unsigned char isSend = 0;

void uart1Write(unsigned char dat)
{
  FIFOBuffer_push(&uartSendFIFO, dat);
}
void uart1WriteBuf(unsigned char *buffer, unsigned char length)
{
  unsigned char i = 0;
  for (i = 0; i < length; i++)
  {
    FIFOBuffer_push(&uartSendFIFO, buffer[i]);
  }
}
void uart1WriteString(unsigned char *str)
{
  while (*str)
  {
    FIFOBuffer_push(&uartSendFIFO, *str++);
  }
}

void initUartCommand(void (*commandCallback)(unsigned char *buffer, unsigned char length))
{
  uartReceiveFIFO.headPos = 0;
  uartReceiveFIFO.tailPos = 0;
  uartReceiveFIFO.bufferSize = UartReceiveSize;
  uartReceiveFIFO.buffer = uartReceiveBuffer;

  uartSendFIFO.headPos = 0;
  uartSendFIFO.tailPos = 0;
  uartSendFIFO.bufferSize = UartSendSize;
  uartSendFIFO.buffer = uartSendBuffer;

  uartCommand.Start1 = 0xf0;
  uartCommand.Start2 = 0xf1;
  uartCommand.End1 = 0xe0;
  uartCommand.End2 = 0xe1;
  uartCommand.isStart = 0;
  uartCommand.count = 0;
  uartCommand.bufferSize = UartCommandBufferSize;
  uartCommand.buffer = uartCommandBuffer;
  uartCommand.resolveCommandCallback = commandCallback;
}
