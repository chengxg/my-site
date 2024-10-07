
// FIFO环形缓冲队列
typedef struct FIFOBuffer {
  unsigned char headPos;    //缓冲区头部位置
  unsigned char tailPos;    //缓冲区尾部位置
  unsigned char bufferSize; // 缓冲区长度
  unsigned char *buffer;    //缓冲区数组
};

unsigned char FIFOBuffer_available(struct FIFOBuffer *fifo_buffer) {
  return fifo_buffer->headPos != fifo_buffer->tailPos;
}

void FIFOBuffer_flush(struct FIFOBuffer *fifo_buffer) {
  fifo_buffer->headPos = 0;
  fifo_buffer->tailPos = 0;
}

unsigned char FIFOBuffer_read(struct FIFOBuffer *fifo_buffer) {
  unsigned char buf = 0;
  //如果头尾接触表示缓冲区为空
  if (fifo_buffer->headPos != fifo_buffer->tailPos) {
    buf = fifo_buffer->buffer[fifo_buffer->headPos];
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize) {
      fifo_buffer->headPos = 0;
    }
  }
  return buf;
}

void FIFOBuffer_push(struct FIFOBuffer *fifo_buffer, unsigned char buf) {
  fifo_buffer->buffer[fifo_buffer->tailPos] = buf;         //从尾部追加
  if (++fifo_buffer->tailPos >= fifo_buffer->bufferSize) { //尾节点偏移
    fifo_buffer->tailPos = 0;
  }
  if (fifo_buffer->tailPos == fifo_buffer->headPos) {
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize) {
      fifo_buffer->headPos = 0;
    }
  }
}

// 检测命令格式 {Start1,Start2, data ,len,End1,End2}
struct MyCommand {
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
#define UartCommandBufferSize 64
uint8 uartCommandBuffer[UartCommandBufferSize] = {0};
struct MyCommand uartCommand;

void MyCommand_addData(struct MyCommand *command, unsigned char tempData) {
  if (!command->isStart) {
    if (command->count == 0 && tempData != command->Start1) {
      return;
    }
    command->count++;
    if (command->count == 2) {
      if (command->Start2 == tempData) {
        command->isStart = true;
        command->count = 0;
      } else {
        command->count = 0;
        if (tempData == command->Start1) {
          command->count++;
        }
      }
    }
    if (command->count > 2) {
      command->count = 0;
      command->isStart = false;
    }
    return;
  }

  if (command->count >= command->bufferSize) {
    command->count = 0;
    command->isStart = false;
  }

  command->buffer[command->count] = tempData;
  command->count++;

  if (command->isStart && command->count >= 4) {
    // 检测结束
    if (tempData == command->End2 &&
        command->buffer[command->count - 2] == command->End1) {
      // 长度位
      if (command->buffer[command->count - 3] == command->count - 3) {
        if (command->resolveCommandCallback) {
          command->resolveCommandCallback(command->buffer, command->count - 3);
        }
        command->isStart = false;
        command->count = 0;
      }
    }
  }
}

#define UartReceiveSize 128
#define UartSendSize 128
uint8 uartReceiveBuffer[UartReceiveSize] = {0};
uint8 uartSendBuffer[UartSendSize] = {0};
struct FIFOBuffer uartReceiveFIFO;
struct FIFOBuffer uartSendFIFO;
uint8 isSend = 0;

void UART1_Isr(void) interrupt 4 {
  if (TI) {
    TI = 0;
    // 队列中还有数据, 继续发送
    if (uartSendFIFO.headPos != uartSendFIFO.tailPos) {
      SBUF = FIFOBuffer_read(&uartSendFIFO);
    } else {
      isSend = 0;
    }
  }
  if (RI) {
    RI = 0;
    // 接收到数据, 放入队列
    FIFOBuffer_push(&uartReceiveFIFO, SBUF);
  }
}
void uart1Write(uint8 dat) {
  FIFOBuffer_push(&uartSendFIFO, dat);
  if (isSend == 0) {
    isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
  }
}
void uart1WriteBuf(uint8 *buffer, uint8 length) {
  uint8 i = 0;
  for (i = 0; i < length; i++) {
    FIFOBuffer_push(&uartSendFIFO, buffer[i]);
  }
  if (isSend == 0) {
    isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
  }
}
void uart1WriteString(uint8 *str) {
  while (*str) {
    FIFOBuffer_push(&uartSendFIFO, *str++);
  }
  if (isSend == 0) {
    isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
  }
}

void initUartCommand(void (*commandCallback)(uint8 *buffer, uint8 length)) {
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