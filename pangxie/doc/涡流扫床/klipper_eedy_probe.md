# 涡流感应探针

本文档介绍如何在 Klipper 中使用
[涡流](https://en.wikipedia.org/wiki/Eddy_current)感应探针。

目前，涡流探针还不能精确执行 Z 轴归位（即 `G28 Z`）。
但该传感器可以精确执行 Z 探测（即 `PROBE ...`）。
更多细节请参见[归位修正](Eddy_Probe.md#homing-correction-macros)。

首先，需要在 printer.cfg 中声明一个
[probe_eddy_current 配置段](Config_Reference.md#probe_eddy_current)。
建议将 `descend_z` 设置为 0.5mm。通常该传感器还需要配置
`x_offset` 和 `y_offset`。如果这些值暂时未知，
应在初始校准阶段先进行估算。

校准的第一步是为传感器确定合适的 DRIVE_CURRENT。
先让打印机归位，然后移动工具头，使传感器位于热床中心附近，
并距离热床大约 20mm。接着执行 `LDC_CALIBRATE_DRIVE_CURRENT
CHIP=<config_name>` 命令。例如，如果配置段名称是
`[probe_eddy_current my_eddy_probe]`，那么应执行
`LDC_CALIBRATE_DRIVE_CURRENT CHIP=my_eddy_probe`。
该命令通常会在几秒钟内完成。完成后，执行 `SAVE_CONFIG`
将结果保存到 printer.cfg，并重启。

涡流传感器本质上被用作接近/距离传感器（类似激光测距尺）。
第二步校准是将传感器读数与对应的 Z 高度建立映射关系。
先让打印机归位，然后移动工具头，使喷嘴位于热床中心附近。
接着运行 `PROBE_EDDY_CURRENT_CALIBRATE CHIP=my_eddy_probe`。
工具启动后，按照[“纸片测试”](Bed_Level.md#the-paper-test)
中的步骤，确定该位置喷嘴与热床之间的实际距离。
完成这些步骤后，可使用 `ACCEPT` 接受当前位置。
随后，工具会移动工具头，使传感器位于先前喷嘴所在点的上方，
并执行一系列运动，将传感器读数与 Z 位置对应起来。
这一过程大约需要几分钟。完成后，工具会输出探针性能数据：
```
probe_eddy_current: noise 0.000642mm, MAD_Hz=11.314 in 2525 queries
Total frequency range: 45000.012 Hz
z: 0.250 # noise 0.000200mm, MAD_Hz=11.000
z: 0.530 # noise 0.000300mm, MAD_Hz=12.000
z: 1.010 # noise 0.000400mm, MAD_Hz=14.000
z: 2.010 # noise 0.000600mm, MAD_Hz=12.000
z: 3.010 # noise 0.000700mm, MAD_Hz=9.000
```
然后执行 `SAVE_CONFIG`，将结果保存到 printer.cfg 并重启。

完成初始校准后，建议进一步确认 `x_offset` 和 `y_offset`
是否准确。请按照[校准探针 X/Y 偏移](Probe_Calibrate.md#calibrating-probe-x-and-y-offsets)
中的步骤进行。如果修改了 `x_offset` 或 `y_offset`，
则务必在修改后重新运行 `PROBE_EDDY_CURRENT_CALIBRATE`
命令（即上述流程）。

校准完成后，就可以使用所有依赖 Z 探针的标准 Klipper 工具。

需要注意的是，涡流传感器（以及一般的感应式探针）
容易受到“热漂移”的影响。也就是说，温度变化会导致
报告的 Z 高度发生变化。无论是热床表面温度变化，
还是传感器硬件本身温度变化，都可能使结果产生偏差。
因此，校准和探测必须在打印机温度稳定时进行。

## 归位修正宏

由于当前实现上的限制，涡流传感器的归位与探测
采用了不同的处理方式。
因此，归位会存在一个偏移误差，
而探测则能正确处理这个问题。

要修正归位偏移，
可以在 homing override 中，
或在起始 G-Code 中使用下面建议的宏。

配置中还必须定义
[force_move](Config_Reference.md#force_move) 段。

```
[gcode_macro _RELOAD_Z_OFFSET_FROM_PROBE]
gcode:
  {% set Z = printer.toolhead.position.z %}
  SET_KINEMATIC_POSITION Z={Z - printer.probe.last_probe_position.z}

[gcode_macro SET_Z_FROM_PROBE]
gcode:
  {% set METHOD = params.METHOD | default("automatic") %}
  PROBE METHOD={METHOD}
  _RELOAD_Z_OFFSET_FROM_PROBE
  G0 Z5
```

## 轻触校准

涡流探针支持一种特殊的探测方式，称为“tap”探测。
这种机制会控制工具头向下移动，直到喷嘴接触热床。
喷嘴与热床接触后，可能会引起传感器测量值的变化，
系统可以检测到这种变化，并以此停止进一步下压。
随后喷嘴会再次抬离热床，系统会分析抬升过程中
传感器测量值的变化，以确定喷嘴与热床脱离接触的位置。

要使用“tap”探测，必须配置 `tap_threshold` 参数。
这个参数用于决定在一次 “tap” 探测中，
工具头向下运动应当在何时被停止。
如果该值过大，可能导致喷嘴已接触热床却未被检测到，
从而使喷嘴继续失控地下压并撞击热床。
如果该值过小，则可能在喷嘴真正接触热床之前就提前停止，
从而导致探测结果严重失真。

可以使用 `PROBE_EDDY_CURRENT_TAP_CALIBRATE` 命令
来配置合适的 `tap_threshold` 值。
该工具应在完成主校准 `PROBE_EDDY_CURRENT_CALIBRATE`
之后再运行。
请按如下步骤校准 `tap_threshold`：

1. 确认喷嘴和热床表面都已清洁。启用打印机并完成归位，
   将工具头移动到热床中心附近的位置，
   并确保喷嘴距离热床约 3 到 10 毫米。

2. 下一步会让喷嘴接触热床。这个过程始终存在一定风险，
   因此如果探测下降在接触热床后仍未停止，
   需要准备随时执行紧急停止（`M112`）。
   准备好后，执行以下命令：
   `PROBE_EDDY_CURRENT_TAP_CALIBRATE TAP=guess`
   该命令会分析主探针校准期间采集的数据，
   先粗略估算一个 tap_threshold，
   然后使用该阈值执行一次对应的 “tap” 探测。
   理想情况下，上述命令会让探针下降直到接触热床，
   然后抬起，并最终报告一个有效的探测结果。
   如果没有成功，请查看本节末尾的排障说明。
   如果本次尝试成功，则继续下一步。

3. 下一步是使用“refined”模式再次运行一次 tap 探测，
   获得更精细的阈值设定。工具会利用之前成功探测时
   收集到的信息来推导这个改进后的阈值。
   请确保喷嘴仍位于热床中心附近，
   并保持在热床上方 3 到 10mm，
   同时做好执行紧急停止的准备，
   然后运行以下命令：
   `PROBE_EDDY_CURRENT_TAP_CALIBRATE TAP=refine`
   理想情况下，该命令也应当成功；若未成功，
   请参考本节末尾的排障说明。
   如果本次尝试成功，则继续下一步。

4. 如果 refined 阈值下的探测成功，下一步要验证它在多次探测中
   是否稳定。请确保喷嘴仍位于热床中心附近，
   且距离热床 3 到 10mm，
   做好执行紧急停止的准备，
   然后运行以下命令：
   `PROBE_EDDY_CURRENT_TAP_CALIBRATE TAP=verify`
   该命令会连续探测热床五次。
   理想情况下，这一步同样应能成功；若未成功，
   请参考本节末尾的排障说明。
   如果成功，则继续下一步。

5. 如果上述所有步骤都成功完成，
   就可以执行 `SAVE_CONFIG`，
   将 `tap_threshold` 参数保存到 printer.cfg 中。
   至此，校准完成。

如果上述任一步骤未能成功，
则可能需要进行排查，并手动确定合适的 `tap_threshold`。
可以通过如下形式的命令进行测试：
`PROBE METHOD=tap TAP_THRESHOLD=<value>`
其中 `<value>` 是待测试的阈值。

通常，如果探测在喷嘴接触热床之前就停止，
说明当前提供的 `TAP_THRESHOLD` 过低。
可尝试将其提高约 10% 后重新测试。
相反，如果喷嘴接触热床后探测仍未停止，
则说明 `TAP_THRESHOLD` 过高。
此时可以考虑把当前尝试值减半。

如果自动校准工具在初始 “guess” 阶段就失败了，
可以把工具报告的 tap_threshold 作为手动测试的起点。
一旦成功完成一次手动探测，
就可以重新回到上面的主流程，并从 “refine” 阶段继续。

## 热漂移校准

与所有感应式探针一样，涡流探针也会受到明显的热漂移影响。
如果该涡流探针在线圈上带有温度传感器，
就可以配置一个 `[temperature_probe]` 来上报线圈温度，
并启用软件热漂移补偿。
要将温度探针与涡流探针关联，
`[temperature_probe]` 配置段必须与
`[probe_eddy_current]` 配置段使用相同的名称。
例如：

```
[probe_eddy_current my_probe]
# 涡流探针配置...

[temperature_probe my_probe]
# 温度探针配置...
```

如何配置 `temperature_probe`，
请参阅[配置参考](Config_Reference.md#temperature_probe)。
建议配置 `calibration_position`、
`calibration_extruder_temp`、`extruder_heating_z` 和
`calibration_bed_temp` 这些选项，
因为这样可以自动化下面流程中的部分步骤。
如果待校准的打印机是封闭式机箱，
强烈建议将 `max_validation_temp` 设置为 100 到 120 之间的某个值。

某些涡流探针厂商可能会提供一组出厂热漂移校准数据，
可手动填写到 `[probe_eddy_current]` 段的 `drift_calibration`
选项中。如果厂商没有提供，或者提供的校准在你的系统上
表现不佳，那么 `temperature_probe` 模块提供了一个
手动校准流程，可通过 `TEMPERATURE_PROBE_CALIBRATE`
这条 G-Code 命令执行。

在开始校准之前，用户应当先大致了解探针线圈所能达到的
最高温度。这个温度应当作为
`TEMPERATURE_PROBE_CALIBRATE` 命令中的 `TARGET` 参数值。
校准的目标是尽可能覆盖更宽的温度区间，
因此理想做法是从冷机开始，
并在线圈升到可达到的最高温度时结束。

配置好 `[temperature_probe]` 后，
可按以下步骤执行热漂移校准：

- 当 `[temperature_probe]` 已配置并完成关联后，
  必须先执行 `PROBE_EDDY_CURRENT_CALIBRATE` 对探针进行校准。
  这样会记录校准时的温度，这是进行热漂移补偿所必需的。
- 确保喷嘴上没有残留碎屑和耗材。
- 在开始校准前，热床、喷嘴和探针线圈都应处于冷态。
- 如果 `[temperature_probe]` 中**没有**配置
  `calibration_position`、`calibration_extruder_temp`
  和 `extruder_heating_z`，则还需要执行以下步骤：
  - 将工具头移动到热床中心位置，且 Z 高度应高于热床 30mm 以上。
  - 将喷嘴加热到高于热床最大安全温度的某个值。
    对大多数配置来说，150 到 170C 已经足够。
    加热喷嘴的目的是避免校准期间喷嘴热膨胀带来影响。
  - 当喷嘴温度稳定后，将 Z 轴下降到距离热床约 1mm 的位置。
- 开始热漂移校准。如果探针名称为 `my_probe`，
  且探针最高可达到温度约为 80C，
  那么应执行的命令是
  `TEMPERATURE_PROBE_CALIBRATE PROBE=my_probe TARGET=80`。
  如果相关参数已配置，工具会自动移动到
  `calibration_position` 指定的 X、Y 坐标，
  并移动到 `extruder_heating_z` 指定的 Z 高度。
  喷嘴加热到指定温度后，工具会再移动到
  `calibration_position` 指定的 Z 值。
- 该过程会请求一次手动探测。
  请使用纸片测试完成手动调平，并执行 `ACCEPT`。
  校准过程会先采集第一组样本，
  然后把探针停放到加热位置。
- 如果**没有**配置 `calibration_bed_temp`，
  则此时需要手动打开热床加热，并升到最大安全温度。
  否则，这一步会自动完成。
- 默认情况下，校准程序会在温度每上升 2C 时
  请求一次新的手动探测，直到达到 `TARGET`。
  采样间隔温差可以通过
  `TEMPERATURE_PROBE_CALIBRATE` 中的 `STEP` 参数自定义。
  设置自定义 `STEP` 时需要谨慎，
  因为值过大可能导致采样点过少，从而使校准质量变差。
- 在热漂移校准期间，还可使用以下额外 G-Code 命令：
  - `TEMPERATURE_PROBE_NEXT`：在尚未达到步进温差时，强制立即采集一个新样本。
  - `TEMPERATURE_PROBE_COMPLETE`：在尚未达到 `TARGET` 时提前结束校准。
  - `ABORT`：终止校准并丢弃结果。
- 当校准完成后，执行 `SAVE_CONFIG` 保存热漂移校准数据。

可以看出，上述校准流程比大多数其他校准过程都更复杂、
也更耗时。为了得到理想的校准效果，
通常需要一定练习，并可能需要尝试多次。

## 错误说明

可能出现的归位错误及处理建议：

- Sensor error
  - 检查日志以获取详细错误信息
- Eddy I2C STATUS/DATA error.
  - 检查接线是否松动
  - 尝试软件 I2C，或降低 I2C 速率
- Invalid read data
  - 处理方式同 I2C 问题

可能出现的传感器错误及处理建议：

- Frequency over valid hard range
  - 检查频率配置
  - 可能为硬件故障
- Frequency over valid soft range
  - 检查频率配置
- Conversion Watchdog timeout
  - 可能为硬件故障

Amplitude Low/High 警告信息可能意味着：

- 传感器离热床过近
- 传感器离热床过远
- 当前温度高于校准时的温度
- 电容缺失

在某些传感器上，Amplitude 警告指示
无法被完全避免。

你可以尝试在工作温度下重新执行
`LDC_CALIBRATE_DRIVE_CURRENT` 校准，
或者在当前校准值基础上将 `reg_drive_current` 提高 1 到 2。

通常来说，这有点像汽车发动机故障灯。
它可能意味着系统存在某种问题。