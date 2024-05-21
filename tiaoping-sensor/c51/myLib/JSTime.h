#define JSTimeSize 20 //初始化定时的个数

unsigned long globeTime = 0;
struct JSTimeStruct {
  unsigned long id;         //定时器id,用来取消定时器
  unsigned long startTime;  // 开始执行的时间
  unsigned long periodTime; // 延时的时间 或者 间隔执行的时间
  void (*callback)();       // 回调函数
};
struct JSTimeStruct JSTime_arr[JSTimeSize];
unsigned long JSTime_createTimeId = 0x80000000;
unsigned long JSTime_createIntervalTimeId = 1;

unsigned long micros() {
  // 系统微秒计时
  // (globeTime*65536+(TH0-0)*256+(TL0-0))/2
  return (globeTime << 15) + ((TH0 << 8 | TL0) >> 1);
}
void T_IRQ0(void) interrupt 1 using 1 { globeTime++; }
void Timer0_Init(void) // 32768微秒@24.000MHz
{
  AUXR &= 0x7F; // 定时器时钟12T模式
  TMOD &= 0xF0; // 设置定时器模式
  TL0 = 0x00;   // 设置定时初始值
  TH0 = 0x00;   // 设置定时初始值
  TF0 = 0;      // 清除TF0标志
  TR0 = 1;      // 定时器0开始计时
  ET0 = 1;      // 打开定时器0中断
  IPH |= 0x02;    // 设置定时器0的中断的优先级为高
  IP |= 0x02;    // 设置定时器0的中断的优先级为高
}

// 定时器初始化函数
void JSTime_init() {
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++) {
    JSTime_arr[i].callback = 0;
    JSTime_arr[i].periodTime = 0;
    JSTime_arr[i].startTime = 0;
    JSTime_arr[i].id = 0;
  }
  Timer0_Init();
}

// loop循环中不断刷新定时器
// JSTime_refresh中的局部变量变成全局变量
// 我发现这几个变量声明为JSTime_refresh函数内局部变量时,
// 在执行JSTime_arr[i].callback()回调函数时(这个回调函数中声明了一个局部变量的数组,
// 并且对这个数组进行赋值时),
// 会出现这几个局部变量的值有可能被莫名奇妙修改的情况, 纳尼??? 这是什么原因呢???
// C51编译器的bug吗? C51是真的难以理解啊!
unsigned long JSTime_cacheId = 0;
unsigned long JSTime_currentTime = 0;
unsigned char JSTime_i = 0;
unsigned char JSTime_isFree = 0;
void JSTime_refresh() {
  JSTime_currentTime = micros();
  for (JSTime_i = 0; JSTime_i < JSTimeSize; JSTime_i++) {
    if (JSTime_arr[JSTime_i].id != 0) {
      if (JSTime_currentTime - JSTime_arr[JSTime_i].startTime >=
          JSTime_arr[JSTime_i].periodTime) {
        JSTime_cacheId = JSTime_arr[JSTime_i].id;
        if (JSTime_cacheId >= 0x80000000) {
          // setTimeout 执行完毕就销毁
          JSTime_isFree = 1;
        } else {
          JSTime_isFree = 0;
          // setInteval 不断进行
          JSTime_arr[JSTime_i].startTime = JSTime_currentTime;
        }
        if (JSTime_arr[JSTime_i].callback) {
          JSTime_arr[JSTime_i].callback();
          JSTime_currentTime = micros();
        }
        // 防止在回调函数里调用了 clearTime 而引发bug
        if (JSTime_isFree == 1 && JSTime_arr[JSTime_i].id == JSTime_cacheId) {
          // setTimeout 执行完毕就销毁
          JSTime_arr[JSTime_i].id = 0;
        }
      }
    }
  }
}

// 延时执行, delayTime单位为ms
unsigned long setTimeout(void (*callback)(), float delayTime) {
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++) {
    // 找出失效的 结构体
    if (JSTime_arr[i].id == 0) {
      JSTime_arr[i].callback = callback;
      JSTime_arr[i].periodTime = delayTime * 1000;
      JSTime_arr[i].startTime = micros();
      if (JSTime_createTimeId > 0xfffffff0) {
        JSTime_createTimeId = 0x80000000;
      }
      JSTime_createTimeId++;
      JSTime_arr[i].id = JSTime_createTimeId;
      return JSTime_createTimeId;
    }
  }
  return 0;
}

// 间隔时间执行, intervalTime单位为ms
unsigned long setInterval(void (*callback)(), float intervalTime) {
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++) {
    // 找出失效的 结构体
    if (JSTime_arr[i].id == 0) {
      JSTime_arr[i].startTime = micros();
      JSTime_arr[i].callback = callback;
      JSTime_arr[i].periodTime = intervalTime * 1000;
      if (JSTime_createIntervalTimeId > 0x7ffffff0) {
        JSTime_createIntervalTimeId = 1;
      }
      JSTime_createIntervalTimeId++;
      JSTime_arr[i].id = JSTime_createIntervalTimeId;
      return JSTime_createIntervalTimeId;
    }
  }
  return 0;
}

// 停止计时
void clearTime(unsigned long timeId) {
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++) {
    if (timeId == JSTime_arr[i].id) {
      JSTime_arr[i].id = 0;
    }
  }
}

// 停止计时
void clearAllTime() {
  unsigned char i = 0;
  for (i = 0; i < JSTimeSize; i++) {
    JSTime_arr[i].id = 0;
  }
}