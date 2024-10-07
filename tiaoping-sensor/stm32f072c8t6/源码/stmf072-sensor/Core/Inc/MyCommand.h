#ifndef __MYCOMMAND_H
#define __MYCOMMAND_H

#ifdef __cplusplus
extern "C"
{
#endif
  // FIFO环形缓冲队列
  struct FIFOBuffer
  {
    unsigned char headPos;    // 缓冲区头部位置
    unsigned char tailPos;    // 缓冲区尾部位置
    unsigned char bufferSize; // 缓冲区长度
    unsigned char *buffer;    // 缓冲区数组
  };

  // 检测命令格式 {Start1,Start2, data ,len,End1,End2}
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

    // 检测命令格式 {Start1,Start2, data... ,len,End1,End2}
    void (*resolveCommandCallback)(unsigned char *buffer, unsigned char length);
  };

  extern struct MyCommand uartCommand;

#define UartReceiveSize 255
#define UartSendSize 255
  extern unsigned char uartReceiveBuffer[UartReceiveSize];
  extern unsigned char uartSendBuffer[UartSendSize];
  extern unsigned char uartSendCacheBuffer[UartSendSize];
  extern struct FIFOBuffer uartReceiveFIFO;
  extern struct FIFOBuffer uartSendFIFO;

  extern unsigned char FIFOBuffer_available(struct FIFOBuffer *fifo_buffer);
  extern unsigned char FIFOBuffer_read(struct FIFOBuffer *fifo_buffer);
  extern void FIFOBuffer_push(struct FIFOBuffer *fifo_buffer, unsigned char buf);
  extern void MyCommand_addData(struct MyCommand *command, unsigned char tempData);
  extern void uart1WriteBuf(unsigned char *buffer, unsigned char length);
  extern void initUartCommand(void (*commandCallback)(unsigned char *buffer, unsigned char length));

#ifdef __cplusplus
}
#endif

#endif /* __MYCOMMAND_H */