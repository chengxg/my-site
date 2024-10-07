#ifndef __JSTIME_H
#define __JSTIME_H

#ifdef __cplusplus
extern "C"
{
#endif

  extern void JSTime_init();
  extern void JSTime_refresh();
  extern unsigned long setTimeout(void (*callback)(), float delayTime);
  extern unsigned long setInterval(void (*callback)(), float intervalTime);
  extern void clearTime(unsigned long timeId);
  extern void clearAllTime();

#ifdef __cplusplus
}
#endif

#endif /* __JSTIME_H */