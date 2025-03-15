/**
 * echarts图表. 高性能显示, 亿级数据展示, 500Kb/s实时数据更新
 * 缩放优化, 鼠标滚轮放大缩小, shift键只放大y轴, 不按shift键放大x轴
 * @author b站-仁泉之子
 * @brief
 * @version 1
 * @date 2025-03-15
 *
 * @copyright Copyright (c) 2025
 */

function useChart() {
  let updateViewTimeId = 0;
  let lastUpdateViewTime = Date.now();
  let chart = null;
  let MaxSenconds = 60; // 显示的最大时间
  let TimeBase = 100 // 每个点的时间间隔us
  let MaxDataSize = MaxSenconds * 1000000 / TimeBase; // 存储深度, 最大数据量, 60s, 每100us一个数据
  let MaxShowDataSize = 1000; // 窗口显示的最大数据, 数据越大, 页面越卡顿
  let cacheData1 = []
  let cacheData2 = []
  let cacheData3 = []
  let cacheDataTime = []

  let chartOptions = {
    animation: false,
    title: {
      text: '{a|实时电压(V)}  {b|滚轮缩放x轴}  {c|shift+滚轮缩放y轴}',
      textStyle: {
        rich: {
          a: {
            color: '#000',
            fontSize: 20,
          },
          b: {
            color: '#f44336',
            fontSize: 14,
          },
          c: {
            color: '#43a047',
            fontSize: 14,
          },
        }
      },
    },
    grid: {
      top: 40,
      left: 60,
      right: 40,
      bottom: 70
    },
    animation: false,
    color: ['#039be5', '#4caf50', '#ff9800', '#f44336'],
    tooltip: {
      animation: false,
      trigger: 'axis',
      alwaysShowContent: false, // 始终显示 tooltip
      axisPointer: {
        axis: 'x',
        type: 'cross',
        snap: true,
        show: true,
        precision: 2,
        label: {
          show: true,
          fontSize: 16,
          formatter: function (params) {
            if (params.axisDimension == 'x') {
              return formatTime(params.value)
            }
            return params.value.toFixed(3) + 'V'
          }
        },
      },
      valueFormatter: function (value, dataIndex) {
        if (value == null) {
          return ''
        }
        return value.toFixed(3) + 'V'
      },
    },
    legend: {
      type: 'plain',
      icon: "rect",
      data: ['gpio', '24V', '5V']
    },
    xAxis: {
      type: 'value',
      splitLine: {
        show: true
      },
      minorTick: {
        show: true
      },
      minorSplitLine: {
        show: true
      },
      min: 0,
      max: MaxDataSize,
      splitNumber: 15,
      axisLabel: {
        color: '#000',
        fontSize: 16,
        margin: 10,
        formatter: function (value, index) {
          return formatTime(value)
        }
      }
    },
    yAxis: {
      type: 'value',
      min: -5,
      max: 30,
      splitNumber: 8,
      splitLine: {
        show: true
      },
      minorTick: {
        show: true
      },
      minorSplitLine: {
        show: false
      },
      axisLabel: {
        color: '#000',
        fontSize: 16,
      }
    },
    series: [{
      type: 'line',
      large: true, // 启用 large 模式
      name: 'gpio',
      clip: true,
      data: [],
      showSymbol: false,
      symbol: "emptyCircle",
      showAllSymbol: 'auto',
      symbolSize: 6,
    }, {
      type: 'line',
      large: true, // 启用 large 模式
      name: '24V',
      clip: true,
      data: [],
      showSymbol: false,
      symbol: "emptyCircle",
      showAllSymbol: 'auto',
      symbolSize: 6,
    }, {
      type: 'line',
      large: true, // 启用 large 模式
      name: '5V',
      clip: true,
      data: [],
      showSymbol: false,
      symbol: "emptyCircle",
      showAllSymbol: 'auto',
      symbolSize: 6,
    }],
    dataZoom: [
      {
        id: 'dataZoomX',
        show: true,
        type: 'inside',
        filterMode: 'none',
        xAxisIndex: [0],
        start: 0,
        end: 100,
        zoomOnMouseWheel: false, // 'shift'
        rangeMode: ['value', 'percent'],
      },
      {
        id: 'dataZoomY',
        show: true,
        type: 'inside',
        filterMode: 'none',
        yAxisIndex: [0],
        start: 0,
        end: 100,
        zoomOnMouseWheel: false, //'ctrl',
      },
      // {
      //   id: 'dataZoomY2',
      //   show: true,
      //   type: 'slider',
      //   filterMode: 'none',
      //   yAxisIndex: [0],
      //   start: 0,
      //   end: 100,
      //   zoomOnMouseWheel: false, //'ctrl',
      // },
    ],
  }

  function initChart() {
    let chartDom = document.getElementById('chart')
    if (!chartDom) {
      return
    }
    chart = echarts.init(chartDom);
    chart.setOption(chartOptions);
    window.addEventListener("resize", () => {
      chart.resize()
    })
    chart.on('datazoom', function (params) {
      if (!params.batch) {
        return
      }
      let zoomX = params.batch.find(item => item.dataZoomId == "dataZoomX")
      if (zoomX) {
        updateRangeData(zoomX.start, zoomX.end)
      }
    });
    chart.getZr().on('mousewheel', handleWheel);
    // testAddChartData()
  }

  function testAddChartData() {
    let testAddTimeId = 0
    let isAddData = false
    let startTime = Date.now()
    // 监听空格建, 暂停和开始测试数据
    window.addEventListener("keydown", (e) => {
      if (e.key == " ") {
        if (testAddTimeId) {
          isAddData = false
          clearInterval(testAddTimeId)
          testAddTimeId = 0
          console.log('暂停测试数据, 用时:', (Date.now() - startTime) / 1000, 's')
        } else {
          addTestData()
          isAddData = true
          startTime = Date.now()
        }
      }
    })

    function addTestData() {
      testAddTimeId = setInterval(() => {
        if (!isAddData) {
          return
        }
        let t = Date.now()
        let arr1 = []
        let arr2 = []
        let arr3 = []
        let sinT = Math.sin(t / 100) * 4
        for (let i = 0; i < 500; i++) {
          // 生成正弦波测试数据, 与时间有关
          arr1.push(Math.random() * 0.5 + 10 + sinT)
          arr2.push(Math.random() * 0.5 + 20 + sinT)
          arr3.push(Math.random() * 0.5 + 14 + sinT)
        }
        addChartData(arr1, arr2, arr3, 20)
      }, 10)
    }
  }

  function formatTime(value) {
    value = Math.floor(value * TimeBase)
    let us = value % 1000
    let ms = Math.floor(value / 1000) % 1000
    let s = Math.floor(value / 1000000)
    let str = ""
    if (us != 0) {
      str += us + 'us\n'
    }
    if (ms != 0) {
      str += ms + 'ms\n'
    }
    if (s != 0) {
      str += s + 's'
    }
    return str
  }

  // 优化鼠标滚轮放大
  function handleWheel(event) {
    //按住shift键只放大y轴, 不按shift键放大x轴
    if (event.event.shiftKey) {
      setZoomY(event)
    } else {
      setZoomX(event)
    }
  }

  function setZoomX(event) {
    const delta = event.wheelDelta; // 获取滚轮滚动的距离
    const zoomScale = delta > 0 ? 0.4 : 1 / 0.4; // 定义缩放比例
    const chartOption = chart.getOption();
    const dataZoomOption = chartOption.dataZoom[0];
    const currentStart = dataZoomOption.start;
    const currentEnd = dataZoomOption.end;
    const x = event.offsetX; // 获取鼠标在图表容器内的位置
    let px = chart.convertFromPixel('xAxis', x)  // 转成百分比型式
    let xStart = chartOption.xAxis[0].min
    let xEnd = chartOption.xAxis[0].max
    let percentage = ((px - xStart) / (xEnd - xStart)) * 100
    if (percentage <= currentStart || percentage >= currentEnd) {
      return
    }
    const currentRange = currentEnd - currentStart;
    const newRange = currentRange * zoomScale; // 计算缩放后的新范围
    const leftPercentage = (percentage - currentStart) / currentRange; // 计算左右比例
    // const rightPercentage = (currentRange - percentage) / currentRange;
    // 根据左右比例缩放
    let newStart = percentage - newRange * leftPercentage;
    let newEnd = percentage + newRange - newRange * leftPercentage;
    let rangeSize = Math.floor(newRange * MaxDataSize / 100)
    if (rangeSize < 50) {
      return
    }
    chart.dispatchAction({
      type: 'dataZoom',
      batch: [{
        type: 'dataZoom',
        start: newStart,
        end: newEnd,
        dataZoomId: "dataZoomX"
      }]
    });
  }

  function setZoomY(event) {
    const delta = event.wheelDelta; // 获取滚轮滚动的距离
    const zoomScale = delta > 0 ? 0.6 : 1 / 0.6; // 定义缩放比例
    const chartOption = chart.getOption();
    const dataZoomOption = chartOption.dataZoom[1];
    const currentStart = dataZoomOption.start;
    const currentEnd = dataZoomOption.end;
    const y = event.offsetY; // 获取鼠标在图表容器内的位置
    let py = chart.convertFromPixel('yAxis', y)  // 转成百分比型式
    let yStart = chartOption.yAxis[0].min
    let yEnd = chartOption.yAxis[0].max
    let yLength = yEnd - yStart
    let percentage = ((py - yStart) / yLength) * 100
    if (percentage <= currentStart || percentage >= currentEnd) {
      return
    }
    const currentRange = currentEnd - currentStart;
    const newRange = currentRange * zoomScale; // 计算缩放后的新范围
    const leftPercentage = (percentage - currentStart) / currentRange; // 计算左右比例
    // const rightPercentage = (currentRange - percentage) / currentRange;
    // 根据左右比例缩放
    let newStart = percentage - newRange * leftPercentage;
    let newEnd = percentage + newRange - newRange * leftPercentage;
    let rangeSize = Math.floor(newRange * (yEnd - yStart) / 0.001 / 100)
    if (rangeSize < 10) {
      return
    }
    chart.dispatchAction({
      type: 'dataZoom',
      batch: [{
        type: 'dataZoom',
        start: newStart,
        end: newEnd,
        dataZoomId: "dataZoomY"
      }]
    });
  }

  function getShowData(mixXPercent, maxXPercent) {
    // 根据zoomX的范围, 获取当前显示的数据
    let minX = Math.floor(mixXPercent * MaxDataSize / 100)
    let maxX = Math.floor(maxXPercent * MaxDataSize / 100)
    let arr1 = []
    let arr2 = []
    let arr3 = []
    let len = cacheData1.length
    if (minX >= len) {
      return [arr1, arr2, arr3]
    }
    if (maxX >= len) {
      maxX = len
    }
    let intervalNum = Math.floor((maxX - minX) / MaxShowDataSize)
    if (intervalNum == 0) {
      intervalNum = 1
    }
    let i = minX
    while (true) {
      arr1.push([i, cacheData1[i]])
      arr2.push([i, cacheData2[i]])
      arr3.push([i, cacheData3[i]])
      i += intervalNum
      if (i >= maxX) {
        break
      }
    }
    return [arr1, arr2, arr3]
  }

  function updateRangeData(start, end) {
    let [arr1, arr2, arr3] = getShowData(start, end)
    chart.setOption({
      series: [
        {
          data: arr1
        },
        {
          data: arr2
        },
        {
          data: arr3
        }
      ],
      dataZoom: [{
        start: start,
        end: end,
      }]
    })
  }

  function throttleUpdateData() {
    let t = Date.now()
    if (t - lastUpdateViewTime > 300) {
      clearTimeout(updateViewTimeId)
      lastUpdateViewTime = t
      updateChartView()
    } else {
      clearTimeout(updateViewTimeId)
      updateViewTimeId = setTimeout(() => {
        lastUpdateViewTime = t
        updateChartView()
      }, 300)
    }
  }

  function updateChartView() {
    if (!chart) {
      return
    }
    let option = chart.getOption()
    let zoomX = option.dataZoom[0]
    updateRangeData(zoomX.start, zoomX.end)
  }

  function addChartData(y1, y2, y3, time = 100) {
    let len = y1.length
    if (time != TimeBase || (cacheData1.length + len) > MaxDataSize) {
      cacheData1 = []
      cacheData2 = []
      cacheData3 = []
      cacheDataTime = []
      TimeBase = time
      MaxDataSize = MaxSenconds * 1000000 / TimeBase
      setTimeout(() => {
        chart.setOption({
          xAxis: {
            min: 0,
            max: MaxDataSize,
          }
        })
      }, 10)
    }
    for (let i = 0; i < len; i++) {
      cacheData1.push(y1[i])
      cacheData2.push(y2[i])
      cacheData3.push(y3[i])
      cacheDataTime.push(time)
    }

    throttleUpdateData()
  }

  return {
    initChart,
    addChartData,
  }
}