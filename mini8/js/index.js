/**
 * 3D打印 STM32高性能迷你8轴主板 电源管理web控制界面
 * 采用串口通信, 通过web控制STM32主板的电源管理, 采样ADC值, 显示波形图
 * 
 * @author b站-仁泉之子
 * @brief
 * @version 1
 * @date 2025-03-15
 *
 * @copyright Copyright (c) 2025
 */

function useSerial() {
  const { ref, reactive } = Vue
  let port = null;
  let writer = null;
  let reader = null;
  let readDataCallback = null; // 读取数据回调
  let serialOpenCallback = null; // 串口打开回调

  let serial = reactive({
    name: "",
    baudRate: 1000000,
    dataBits: 8, // 7 or 8
    stopBits: 1, // 1 or 2
    parity: "none", // "none", "even", "odd
    bufferSize: 1024 * 10,
    state: "", // closed, opening, opened
    suport: "serial" in navigator,
    open: false, // 串口是否打开
    stop: false, // 是否停止读取数据
  })

  let serialOptions = reactive({
    baudRateOptions: [
      { label: 500000, value: 500000 },
      { label: 1000000, value: 1000000 },
      { label: 2000000, value: 2000000 },
    ],
    dataBitsOptions: [
      { label: 7, value: 7 },
      { label: 8, value: 8 },
    ],
    stopBitsOptions: [
      { label: 1, value: 1 },
      { label: 2, value: 2 },
    ],
    parityOptions: [
      { label: "无", value: "none" },
      { label: "奇", value: "even" },
      { label: "偶", value: "odd" },
    ],
  })

  async function initPort() {
    let cacheCOMInfo = localStorage.getItem("COMInfo_8zhuban")
    if (cacheCOMInfo) {
      let cacheCOMInfoObj = JSON.parse(cacheCOMInfo)
      serial.name = cacheCOMInfoObj.name
      serial.dataBits = cacheCOMInfoObj.dataBits
      serial.stopBits = cacheCOMInfoObj.stopBits
      serial.parity = cacheCOMInfoObj.parity
    }
  }

  function saveCOMInfo() {
    let cacheCOMInfo = {
      name: serial.name,
      baudRate: serial.baudRate,
      dataBits: serial.dataBits,
      stopBits: serial.stopBits,
      parity: serial.parity,
      bufferSize: serial.bufferSize
    }
    localStorage.setItem("COMInfo_8zhuban", JSON.stringify(cacheCOMInfo))
  }

  async function openPort(isReOpen = true) {
    if (!("serial" in navigator)) {
      return
    }
    if (!port || isReOpen) {
      // 获取用户之前授予该网站访问权限的所有串口。
      port = await navigator.serial.requestPort();
    }

    // 等待串口打开
    await port.open({
      baudRate: serial.baudRate,
      dataBits: serial.dataBits,
      stopBits: serial.stopBits,
      parity: serial.parity,
      bufferSize: serial.bufferSize,
    });
    writer = await port.writable.getWriter()
    reader = await port.readable.getReader()
    serial.open = true
    serial.stop = false
    saveCOMInfo()
    serialOpenCallback && serialOpenCallback()
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) {
          writer.releaseLock();
          reader.releaseLock();
          break;
        }
        if (!serial.stop) {
          readDataCallback && readDataCallback(value)
        }
      }
    } catch (error) {
      console.log(error)
    } finally {
      writer.releaseLock();
      reader.releaseLock();
    }
  }

  function setReadDataCallback(callback) {
    readDataCallback = callback
  }

  function setSerialOpenCallback(callback) {
    serialOpenCallback = callback
  }

  async function closePort() {
    if (port) {
      serial.open = false;
      serial.stop = true;
      try {
        await writer.abort();
        await reader.cancel();
        await port.close();
      } catch (error) {
        console.log(error);
      } finally {
        writer.releaseLock();
        reader.releaseLock();
      }
    }
  }

  function serialWriteData(data) {
    if (writer) {
      writer.write(data)
    }
  }

  function serialToggleStop() {
    serial.stop = !serial.stop
  }

  return {
    serial,
    serialOptions,
    initPort,
    openPort,
    closePort,
    serialWriteData,
    serialToggleStop,
    setReadDataCallback,
    setSerialOpenCallback
  }
}
function useMyCommand() {
  let command = {
    Start1: 0xf2,
    Start2: 0xf3,
    End1: 0xe2,
    End2: 0xe3,
    isStart: false,
    count: 0,
    bufferSize: 255,
    buffer: new Array(255),
  }
  let commandCallback = null;

  function addCommandData(tempData) {
    if (!command.isStart) {
      if (command.count == 0 && tempData != command.Start1) {
        return;
      }
      command.count++;
      if (command.count == 2) {
        if (command.Start2 == tempData) {
          command.isStart = true;
          command.count = 0;
        } else {
          command.count = 0;
          if (tempData == command.Start1) {
            command.count++;
          }
        }
      }
      if (command.count > 2) {
        command.count = 0;
        command.isStart = false;
      }
      return;
    }
    if (command.count > 255 || command.count >= command.bufferSize) {
      command.count = 0;
      command.isStart = false;
      return;
    }

    command.buffer[command.count] = tempData;
    command.count++;

    if (command.isStart && command.count >= 4) {
      // 检测结束
      if (tempData == command.End2 && command.buffer[command.count - 2] == command.End1) {
        // 长度位
        if (command.buffer[command.count - 3] == command.count - 3) {
          if (typeof commandCallback == "function") {
            commandCallback(command.buffer, command.count);
          }
          command.isStart = false;
          command.count = 0;
        }
      }
    }
  }

  function getCheckSum(buffer, start, end) {
    let i = 0;
    let sum = 0;
    for (i = start; i < end; i++) {
      sum += buffer[i];
    }
    return sum % 256;
  }

  function createdCommandData(dataArr) {
    let len = dataArr.length;
    let buffer = new Uint8Array(len + 6);
    let i = 0;
    buffer[i++] = command.Start1;
    buffer[i++] = command.Start2;
    for (let j = 0; j < len; j++) {
      buffer[i++] = dataArr[j];
    }
    buffer[i++] = getCheckSum(buffer, 2, i)
    buffer[i] = i - 2;
    i++;
    buffer[i++] = command.End1;
    buffer[i++] = command.End2;
    return buffer;
  }

  function setResolveCommandCallback(callback) {
    commandCallback = callback
  }

  return {
    addCommandData,
    createdCommandData,
    setResolveCommandCallback
  }
}

(function init() {
  const { createApp, ref, reactive, onMounted, computed, onUnmounted } = Vue
  const hexStrMap = [];
  // 8位整数转16进制字符串
  function int8ToHexString(num) {
    if (hexStrMap.length == 0) {
      for (let i = 0; i < 256; i++) {
        let iHex = i.toString(16).toUpperCase();
        if (iHex.length == 1) {
          iHex = "0" + iHex;
        }
        hexStrMap[i] = iHex;
      }
    }
    num = Math.floor(num);
    return hexStrMap[num & 0xFF];
  }
  const app = createApp({
    name: "app",
    components: {},
    setup() {
      let {
        serial,
        serialOptions,
        initPort,
        openPort,
        closePort,
        serialWriteData,
        serialToggleStop,
        setReadDataCallback,
        setSerialOpenCallback,
      } = useSerial()
      let {
        addCommandData,
        createdCommandData,
        setResolveCommandCallback
      } = useMyCommand()
      let { initChart, addChartData } = useChart()
      // 设备状态
      let state = reactive({
        version: 1, // 版本号
        adc1ToVoltageRatio: 1 / 4096 * 18.8, // adc1电压比例 V/18.8 = adcValue/4096
        adc2ToVoltageRatio: 1 / 4096 * 36.3, // V/36.3 = adcValue/4096
        adc3ToVoltageRatio: 1 / 4096 * 18.8, // V/18.8 = adcValue/4096
        curTime: 0, // 单片机时间 每个计数65536us
        curTimeUs: 0, // 单片机时间 当前定时器0计数值, 1us
        formatCurTime: formatTime(0), // 单片机时间 格式化变成 hh:mm:ss
        sampleChannel: 0, // 采样通道 掩码: 1: GPIO保护 2: 24V 4: 5V
        isProtect: false, // 是否进入保护状态
        SHUT_24V_PIN: 0, // 24V关机引脚
        N_SHUT_24V_PIN: 0, // 泄放引脚
        SHUT_5V_PIN: 0, // 5V关机引脚
        loopCount: 0, // 一定时间内的循环次数
        maxLoopTime: 0, // 一定时间内的最大单次循环时间 us
        debugInt1: 0, // 测试值
        debugInt2: 0, // 测试值

        loopRate: "0", // loop循环速率 (curTime - lastTime) / (loopCount - lastLoopCount)
        adc1FormatValue: "0", // GPIO当前电压值 V
        adc2FormatValue: "0", // 24V当前电压值 V
        adc3FormatValue: "0", // 5V当前电压值 V
        adc1SampleEnable: true, // GPIO保护采样使能
        adc2SampleEnable: true, // 24V采样使能
        adc3SampleEnable: true, // 5V采样使能
        TrigADCValue: 0, // gpio过压保护触发阈值
        TrigFormatValue: "0",// gpio过压保护触发阈值 V
        inputTrigValue: "0", // 临时输入值
        TrigAdcOptions: [
          { label: '3.6V', value: "3.6" },
          { label: '3.8V', value: "3.8" },
          { label: '4.0V', value: "4.0" },
          { label: '4.2V', value: "4.2" },
          { label: '4.4V', value: "4.4" },
          { label: '4.6V', value: "4.6" },
        ],

        loading: false,
        triggerSendADCTimeId: 0,
        adcSampleInterval: 1, // 每隔多少毫秒采样一次数据
        ADCSampleIntervalOptions: [
          { label: "50us", value: 0.05 },
          { label: "100us", value: 0.1 },
          { label: "200us", value: 0.2 },
          { label: "500us", value: 0.5 },
          { label: "1ms", value: 1 },
          { label: "2ms", value: 2 },
        ],
        isStartSample: true, // 是否开始采样
        isChangeBaudRate: false, // 是否更改波特率
        changeBaudRateTimeoutId: 0, // 更改波特率超时定时器
      })

      let commandMethods = {
        setAutoSyncDeiveStatus() {
          setTimeout(() => {
            commandMethods.getDeviceParams()
          }, 100)
          setTimeout(() => {
            commandMethods.updateADCSampleParams()
          }, 200)
          clearInterval(state.triggerSendADCTimeId)
          state.triggerSendADCTimeId = setInterval(() => {
            commandMethods.updateADCSampleParams()
          }, 1000)
        },
        rebootChip() {
          if (!serial.open) {
            return
          }
          let buffer = [
            0x06, 0x07,
          ]
          //重启芯片
          let sendBuffer = createdCommandData(buffer)
          serialWriteData(sendBuffer)
        },
        ackRebootChip() {
          antd.message.success("设置重启成功, 10s之后重启复位!")
        },
        updateUart1Baud(baudRate) {
          if (!serial.open) {
            return
          }
          if (state.isChangeBaudRate) {
            return
          }
          state.isChangeBaudRate = true
          let originalBaudRate = serial.baudRate
          antd.Modal.confirm({
            title: "提示",
            content: "更改波特率后, 10s之内通信不成功, 将恢复原波特率!",
            okText: '确定',
            cancelText: '取消',
            onCancel() {
              state.isChangeBaudRate = false
            },
            onOk: () => {
              baudRate = Number(baudRate)
              serial.baudRate = baudRate
              let buffer = [
                0x08, 0x09,
                (baudRate & 0xff000000) >> 24,
                (baudRate & 0x00ff0000) >> 16,
                (baudRate & 0x0000ff00) >> 8,
                baudRate & 0x00ff
              ]
              //更改串口1波特率
              let sendBuffer = createdCommandData(buffer)
              serialWriteData(sendBuffer)
              state.changeBaudRateTimeoutId = setTimeout(() => {
                serial.baudRate = originalBaudRate
                closePort()
                setTimeout(() => {
                  openPort(false)
                  antd.Modal.error({
                    title: '设置失败',
                    content: '设置失败! 已恢复原1M波特率!',
                  });
                }, 100)
              }, 10000)
              setTimeout(() => {
                serial.baudRate = baudRate
                closePort()
                setTimeout(() => {
                  openPort(false)
                }, 1000)
              }, 50)
            }
          })
        },
        updateADCSampleParams() {
          if (!serial.open) {
            return
          }
          let adcSampleInterval = state.adcSampleInterval * 1000 // us
          let sampleChannelVal = 0
          if (state.isStartSample) {
            if (state.adc1SampleEnable) {
              sampleChannelVal |= 0x01
            }
            if (state.adc2SampleEnable) {
              sampleChannelVal |= 0x02
            }
            if (state.adc3SampleEnable) {
              sampleChannelVal |= 0x04
            }
          }
          let buffer = [
            0x01, 0x02,
            // 发送的时间间隔
            Math.floor(adcSampleInterval / 256),
            adcSampleInterval % 256,
            sampleChannelVal
          ]
          //触发自动发送当前值
          // f0 f1 01 02 00 00 05 e0 e1
          let sendBuffer = createdCommandData(buffer)
          serialWriteData(sendBuffer)
        },
        updateADCSample(dataArr) {
          // 赋值到js变量里
          let i = 1;
          let channel = dataArr[i++];
          let size = dataArr[i++];
          let adc1Arr = []
          let adc2Arr = []
          let adc3Arr = []
          let isChannel1 = channel & 0x01;       // 是否包含通道1
          let isChannel2 = channel & 0x02;       // 是否包含通道2
          let isChannel3 = channel & 0x04;       // 是否包含通道3
          for (let j = 0; j < size; j++) {
            if (isChannel1) {
              adc1Arr.push(((dataArr[i++] << 8) + dataArr[i++]) * state.adc1ToVoltageRatio)
            } else {
              adc1Arr.push(-2)
            }
            if (isChannel2) {
              adc2Arr.push(((dataArr[i++] << 8) + dataArr[i++]) * state.adc2ToVoltageRatio)
            } else {
              adc2Arr.push(-2)
            }
            if (isChannel3) {
              adc3Arr.push(((dataArr[i++] << 8) + dataArr[i++]) * state.adc3ToVoltageRatio)
            } else {
              adc3Arr.push(-2)
            }
          }
          // 添加到图表里
          addChartData(adc1Arr, adc2Arr, adc3Arr, state.adcSampleInterval * 1000)
        },
        updateDeviceParams(dataArr) {
          let i = 2;
          let version = dataArr[i++];
          let TrigADCValue = (dataArr[i++] << 8) + dataArr[i++];
          let sampleChannel = dataArr[i++];
          let isProtect = dataArr[i++];
          let SHUT_24V_PIN = dataArr[i++];
          let N_SHUT_24V_PIN = dataArr[i++];
          let SHUT_5V_PIN = dataArr[i++];
          let adc1Value = (dataArr[i++] << 8) + dataArr[i++];
          let adc2Value = (dataArr[i++] << 8) + dataArr[i++];
          let adc3Value = (dataArr[i++] << 8) + dataArr[i++];
          let curTime = (dataArr[i++] << 24) + (dataArr[i++] << 16) + (dataArr[i++] << 8) + dataArr[i++];
          let curTimeUs = (dataArr[i++] << 8) + dataArr[i++];
          let loopCount = (dataArr[i++] << 24) + (dataArr[i++] << 16) + (dataArr[i++] << 8) + dataArr[i++];
          let maxLoopTime = (dataArr[i++] << 24) + (dataArr[i++] << 16) + (dataArr[i++] << 8) + dataArr[i++];
          let debugInt1 = (dataArr[i++] << 24) + (dataArr[i++] << 16) + (dataArr[i++] << 8) + dataArr[i++];
          let debugInt2 = (dataArr[i++] << 24) + (dataArr[i++] << 16) + (dataArr[i++] << 8) + dataArr[i++];

          state.version = version
          state.TrigADCValue = TrigADCValue
          state.sampleChannel = sampleChannel
          state.isProtect = isProtect
          state.SHUT_24V_PIN = SHUT_24V_PIN
          state.N_SHUT_24V_PIN = N_SHUT_24V_PIN
          state.SHUT_5V_PIN = SHUT_5V_PIN
          // Use BigInt to handle large integer calculations, avoiding 32-bit integer limits
          const previousTimeUs = BigInt(state.curTime) * BigInt(65536) + BigInt(state.curTimeUs);
          const currentTimeUs = BigInt(curTime) * BigInt(65536) + BigInt(curTimeUs);
          let timeDiffUs = Number(currentTimeUs - previousTimeUs);
          state.loopRate = ((loopCount - state.loopCount) / (timeDiffUs / 1000)).toFixed(2)
          state.loopCount = loopCount
          state.maxLoopTime = maxLoopTime
          state.curTime = curTime
          state.curTimeUs = curTimeUs
          state.formatCurTime = formatTime(curTime / (1000 / 65.536))
          state.debugInt1 = debugInt1
          state.debugInt2 = debugInt2

          state.adc1FormatValue = formatValue(adc1Value * state.adc1ToVoltageRatio)
          state.adc2FormatValue = formatValue(adc2Value * state.adc2ToVoltageRatio)
          state.adc3FormatValue = formatValue(adc3Value * state.adc3ToVoltageRatio)
          state.TrigFormatValue = formatValue(Math.round(TrigADCValue / 4096 * 18.8 * 10) / 10)
          state.inputTrigValue = state.TrigFormatValue

          clearTimeout(state.changeBaudRateTimeoutId)
          state.isChangeBaudRate = false
        },
        writeDeviceParams() {
          if (!serial.open) {
            return
          }
          let inputTrigADCValue = Math.round(Number(state.inputTrigValue) / 18.8 * 4096) // 转成adc值
          let buffer = [0x03, 0x02]
          let i = 2;
          buffer[i++] = 0x01
          buffer[i++] = inputTrigADCValue >> 8
          buffer[i++] = inputTrigADCValue & 0xff
          // 设置配置参数
          // f0 f1 03 02 ... e0 e1
          serialWriteData(createdCommandData(buffer))
        },
        getDeviceParams() {
          if (!serial.open) {
            return
          }
          let buffer = [0x02, 0x02]
          // 读取配置参数
          // f0 f1 02 02 ... e0 e1
          serialWriteData(createdCommandData(buffer))
        },
        control5VPower() {
          if (!serial.open) {
            return
          }
          if (state.loading) {
            return
          }
          state.loading = true
          setTimeout(() => {
            state.loading = false
          }, 700)
          let buffer = [0x04, 0x02]
          let i = 2;
          buffer[i++] = state.SHUT_5V_PIN ? 0 : 1
          buffer[i++] = 2
          buffer[i++] = 2
          // 设置配置参数
          // f0 f1 04 02 ... e0 e1
          serialWriteData(createdCommandData(buffer))
        },
        control24VPower() {
          if (!serial.open) {
            return
          }
          if (state.loading) {
            return
          }
          state.loading = true
          setTimeout(() => {
            state.loading = false
          }, 700)
          let buffer = [0x04, 0x02]
          let i = 2;
          buffer[i++] = 2
          buffer[i++] = state.SHUT_24V_PIN ? 0 : 1
          buffer[i++] = 2
          // 设置配置参数
          // f0 f1 04 02 ... e0 e1
          serialWriteData(createdCommandData(buffer))
        },
        controlProtect() {
          if (!serial.open) {
            return
          }
          if (state.loading) {
            return
          }
          state.loading = true
          setTimeout(() => {
            state.loading = false
          }, 700)
          let buffer = [0x04, 0x02]
          let i = 2;
          buffer[i++] = 2
          buffer[i++] = 2
          buffer[i++] = 0
          // 设置配置参数
          // f0 f1 04 02 ... e0 e1
          serialWriteData(createdCommandData(buffer))
        },
      }

      let updateMethods = {
        updateIsStartSample() {
          state.isStartSample = !state.isStartSample
          commandMethods.updateADCSampleParams()
        },
        updateSampleInterval(value) {
          if (state.loading) {
            return
          }
          state.loading = true
          setTimeout(() => {
            state.loading = false
          }, 200)
          state.adcSampleInterval = value
          commandMethods.updateADCSampleParams()
        },
        updateTrigADCValue(value) {
          if (state.loading) {
            return
          }
          state.loading = true
          setTimeout(() => {
            state.loading = false
          }, 500)
          state.inputTrigValue = value
          commandMethods.writeDeviceParams()
        },
      }

      function serialToggleOpen() {
        if (serial.open) {
          closePort()
        } else {
          openPort()
        }
      }

      function formatValue(value, fixed = 1) {
        return Number(value).toFixed(fixed)
      }

      function formatTime(time) {
        time = Math.floor(time)
        let hour = Math.floor(time / 3600)
        let minute = Math.floor((time % 3600) / 60)
        let second = Math.floor(time % 60)
        if (hour < 10) {
          hour = "0" + hour
        }
        if (minute < 10) {
          minute = "0" + minute
        }
        if (second < 10) {
          second = "0" + second
        }
        return `${hour}:${minute}:${second}`
      }

      onMounted(() => {
        initPort()
        initChart()
        setSerialOpenCallback(() => {
          commandMethods.setAutoSyncDeiveStatus()
          state.isStartSample = true
        })
        setReadDataCallback((uint8Value) => {
          let arr = []
          for (let i = 0; i < uint8Value.length; i++) {
            arr.push(int8ToHexString(uint8Value[i]))
            addCommandData(uint8Value[i])
          }
        })
        setResolveCommandCallback((dataArr, length) => {
          // 确认是否是正确的数据
          if (dataArr[0] == 0x01 && dataArr[1] == 0x01) {
            commandMethods.ackRebootChip(dataArr)
          }
          // 当前配置参数
          if (dataArr[0] == 0x02 && dataArr[1] == 0x01) {
            commandMethods.updateDeviceParams(dataArr)
          }
          // 当前ADC读数值
          if (dataArr[0] == 0x20) {
            commandMethods.updateADCSample(dataArr)
          }
        })

        // 监听空格建, 暂停和开始显示波形图
        window.addEventListener("keydown", onkeydown)
      })

      onUnmounted(() => {
        clearInterval(state.triggerSendADCTimeId)
        window.removeEventListener("keydown", onkeydown)
      })

      function onkeydown(e) {
        if (e.key == " ") {
          state.isStartSample = !state.isStartSample
          commandMethods.updateADCSampleParams()
        }
      }

      return {
        serial,
        serialOptions,
        serialToggleStop,
        serialToggleOpen,
        state,
        commandMethods,
        updateMethods,
        formatValue,
        getPopupContainer: () => document.getElementById("app"),
      }
    },
  })
  app.use(antd)
  app.mount("#app")
})()