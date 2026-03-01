# 涡流感应探针

本文档描述了如何在 Klipper 中使用[涡流](https://en.wikipedia.org/wiki/Eddy_current)感应探针。

目前，涡流感应探针不能精确地进行 Z 轴归位（即 `G28 Z`）。但该传感器可以精确地进行 Z 轴探测（即 `PROBE ...`）。有关更多详细信息，请查看[归位校正](Eddy_Probe.md#homing-correction-macros)部分。

首先，在 printer.cfg 文件中声明一个[probe_eddy_current 配置部分](Config_Reference.md#probe_eddy_current)。建议将 `z_offset` 设置为 0.5mm。传感器通常需要 `x_offset` 和 `y_offset`。如果这些值未知，应在初始校准期间估计这些值。

校准的第一步是确定传感器的适当 DRIVE_CURRENT。使打印机归位并导航工具头，使传感器位于床的中心附近，距离床约 20mm。然后发出 `LDC_CALIBRATE_DRIVE_CURRENT CHIP=<config_name>` 命令。例如，如果配置部分名为 `[probe_eddy_current my_eddy_probe]`，则应运行 `LDC_CALIBRATE_DRIVE_CURRENT CHIP=my_eddy_probe`。此命令应在几秒钟内完成。完成后，发出 `SAVE_CONFIG` 命令将结果保存到 printer.cfg 并重新启动。

涡流用作接近/距离传感器（类似于激光尺）。校准的第二步是将传感器读数与相应的 Z 高度相关联。使打印机归位并导航工具头，使喷嘴位于床的中心附近。然后运行 `PROBE_EDDY_CURRENT_CALIBRATE CHIP=my_eddy_probe` 命令。工具启动后，按照["纸张测试"](Bed_Level.md#the-paper-test)中描述的步骤确定给定位置喷嘴和床之间的实际距离。完成这些步骤后，可以 `ACCEPT` 该位置。然后工具将移动工具头，使传感器位于喷嘴曾经所在的点上方，并运行一系列移动以将传感器与 Z 位置相关联。这将需要几分钟时间。工具完成后，它将输出传感器性能数据：
```
probe_eddy_current: noise 0.000642mm, MAD_Hz=11.314 in 2525 queries
Total frequency range: 45000.012 Hz
z_offset: 0.250 # noise 0.000200mm, MAD_Hz=11.000
z_offset: 0.530 # noise 0.000300mm, MAD_Hz=12.000
z_offset: 1.010 # noise 0.000400mm, MAD_Hz=14.000
z_offset: 2.010 # noise 0.000600mm, MAD_Hz=12.000
z_offset: 3.010 # noise 0.000700mm, MAD_Hz=9.000
```
发出 `SAVE_CONFIG` 命令将结果保存到 printer.cfg 并重新启动。

初始校准后，最好验证 `x_offset` 和 `y_offset` 是否准确。按照步骤[校准探针 x 和 y 偏移](Probe_Calibrate.md#calibrating-probe-x-and-y-offsets)。如果修改了 `x_offset` 或 `y_offset`，请确保在更改后运行 `PROBE_EDDY_CURRENT_CALIBRATE` 命令（如上所述）。

校准完成后，可以使用所有使用 Z 探针的标准 Klipper 工具。

请注意，涡流传感器（以及一般的感应探针）容易受到"热漂移"的影响。也就是说，温度变化可能导致报告的 Z 高度发生变化。床表面温度或传感器硬件温度的变化都会使结果产生偏差。重要的是，校准和探测只能在打印机温度稳定时进行。

## 归位校正宏

由于当前的限制，涡流传感器的归位和探测实现方式不同。因此，归位存在偏移误差，而探测则正确处理这一点。

要校正归位偏移，可以在归位覆盖或起始 G 代码中使用建议的宏。

必须在配置中定义[强制移动](Config_Reference.md#force_move)部分。

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

涡流探针测量线圈的共振频率。通过频率的绝对值和 `PROBE_EDDY_CURRENT_CALIBRATE` 的校准曲线，因此可以在没有物理接触的情况下检测床的位置。

利用相同的知识，我们知道频率会随距离变化。可以实时跟踪这种变化并检测接触发生的时间/位置 - 频率变化开始以不同的方式变化。例如，由于碰撞而停止变化。

由于涡流输出并不完美：存在传感器噪声、机械振动、热膨胀和其他差异，因此需要为您的机器校准停止阈值。实际上，它确保涡流的输出数据绝对值每秒变化（速度）足够高 - 高于噪声水平，并且在碰撞时它总是至少减少这个值。

```
[probe_eddy_current my_probe]
# eddy probe configuration...
# Recommended starting values for the tap
#samples: 3
#samples_tolerance: 0.025
#samples_tolerance_retries: 3
tap_threshold: 0 # 0 means tap is disabled
```

在将其设置为任何其他值之前，必须安装 `scipy`：

```bash
~/klippy-env/bin/pip install scipy
```

建议的校准程序如下：
1. 归位 Z 轴
2. 将工具头放置在床的中心。
3. 将 Z 轴移远（例如 30 mm）。
4. 运行 `PROBE METHOD=tap`
5. 如果它在碰撞前停止，增加 `tap_threshold`。

重复直到喷嘴轻轻接触床。这在喷嘴干净且通过视觉检查过程时更容易做到。

您可以通过将工具头放置在中心一次来简化过程。然后，在配置重启时，欺骗机器认为 Z 已归位。
```
SET_KINEMATIC_POSITION X=<middle> Y=<middle> Z=0
G0 Z5 # Optional retract
PROBE METHOD=tap
```

以下是要测试的阈值示例序列：
```
1 -> 5 -> 10 -> 20 -> 40 -> 80 -> 160
160 -> 120 -> 100
```
您的值通常在这些之间。
- 过高的值会为早期碰撞留下较少的安全余量 - 如果在喷嘴和床之间有东西，或者在轻触前喷嘴离床太近。
- 过低 - 可能会因为噪声而使工具头在半空中停止。

您可以通过分析自己的校准程序输出来估计初始阈值：
```
probe_eddy_current: noise 0.000642mm, MAD_Hz=11.314
...
z_offset: 1.010 # noise 0.000400mm, MAD_Hz=14.000
```
估计值为：
```
MAD_Hz * 2
11.314 * 2 = 22.628
```

要进一步微调阈值，可以使用 `PROBE_ACCURACY METHOD=tap`。在默认探针速度为 5 mm/s 的情况下，范围预计约为 0.02 mm。线圈温度升高可能会增加噪声，可能需要额外调整。

您可以通过测量初始校准指南中的纸张厚度来验证轻触精度。预计约为 0.1mm。

轻触精度受采样频率和下降速度的限制。如果您每秒拍摄 24 张移动火车的照片，您只能估计火车在照片之间的位置。

可以降低下降速度。这可能需要降低绝对 `tap_threshold` 值。

只要在传感器的灵敏度范围内，就可以在非导电表面上轻触，只要其后面有金属。最大距离可以近似为线圈最窄部分的约 1.5 倍。

## 热漂移校准

与所有感应探针一样，涡流探针会受到显著的热漂移影响。如果涡流探针在线圈上有温度传感器，则可以配置 `[temperature_probe]` 来报告线圈温度并启用软件漂移补偿。要将温度探针链接到涡流探针，`[temperature_probe]` 部分必须与 `[probe_eddy_current]` 部分共享一个名称。例如：

```
[probe_eddy_current my_probe]
# eddy probe configuration...

[temperature_probe my_probe]
# temperature probe configuration...
```

有关如何配置 `temperature_probe` 的更多详细信息，请参阅[配置参考](Config_Reference.md#temperature_probe)。建议配置 `calibration_position`、`calibration_extruder_temp`、`extruder_heating_z` 和 `calibration_bed_temp` 选项，因为这样可以自动执行下面概述的一些步骤。如果要校准的打印机是封闭的，强烈建议将 `max_validation_temp` 选项设置为 100 到 120 之间的值。

涡流探针制造商可能会提供可手动添加到 `[probe_eddy_current]` 部分的 `drift_calibration` 选项的库存漂移校准。如果他们不提供，或者如果库存校准在您的系统上表现不佳，`temperature_probe` 模块通过 `TEMPERATURE_PROBE_CALIBRATE` gcode 命令提供手动校准程序。

在执行校准之前，用户应该了解可达到的最高温度探针线圈温度。此温度应用于设置 `TEMPERATURE_PROBE_CALIBRATE` 命令的 `TARGET` 参数。目标是在尽可能宽的温度范围内进行校准，因此最好从打印机冷态开始，以线圈达到的最高温度结束。

配置 `[temperature_probe]` 后，可以采取以下步骤执行热漂移校准：

- 当配置并链接 `[temperature_probe]` 时，必须使用 `PROBE_EDDY_CURRENT_CALIBRATE` 校准探针。这会捕获校准期间的温度，这对于执行热漂移补偿是必要的。
- 确保喷嘴没有碎屑和 filament。
- 校准前，床、喷嘴和探针线圈应处于冷态。
- 如果未配置 `[temperature_probe]` 中的 `calibration_position`、`calibration_extruder_temp` 和 `extruder_heating_z` 选项，则需要以下步骤：
  - 将工具移动到床的中心。Z 应高于床 30mm+。
  - 将挤出机加热到高于最高安全床温度的温度。对于大多数配置，150-170C 应该足够。加热挤出机的目的是避免校准期间喷嘴膨胀。
  - 当挤出机温度稳定后，将 Z 轴向下移动到床上方约 1mm 处。
- 开始漂移校准。如果探针的名称是 `my_probe`，我们可以达到的最高探针温度是 80C，则适当的 gcode 命令是 `TEMPERATURE_PROBE_CALIBRATE PROBE=my_probe TARGET=80`。如果配置，工具将移动到 `calibration_position` 指定的 X,Y 坐标和 `extruder_heating_z` 指定的 Z 值。将挤出机加热到指定温度后，工具将移动到 `calibration_position` 指定的 Z 值。
- 该过程将请求手动探针。使用纸张测试和 `ACCEPT` 执行手动探针。校准过程将使用探针进行第一组采样，然后将探针停放在加热位置。
- 如果未配置 `calibration_bed_temp`，请将床加热到最高安全温度。否则，此步骤将自动执行。
- 默认情况下，校准过程将在样本之间每 2C 请求一次手动探针，直到达到 `TARGET`。样本之间的温度增量可以通过在 `TEMPERATURE_PROBE_CALIBRATE` 中设置 `STEP` 参数来自定义。设置自定义 `STEP` 值时应小心，值过高可能会请求太少的样本，导致校准不佳。
- 漂移校准期间可用以下附加 gcode 命令：
  - `TEMPERATURE_PROBE_NEXT` 可用于在达到步长增量之前强制获取新样本。
  - `TEMPERATURE_PROBE_COMPLETE` 可用于在达到 `TARGET` 之前完成校准。
  - `ABORT` 可用于结束校准并丢弃结果。
- 校准完成后，使用 `SAVE_CONFIG` 存储漂移校准。

正如人们可能得出的结论，上面概述的校准过程比大多数其他程序更具挑战性和耗时。可能需要练习和多次尝试才能实现最佳校准。

## 错误描述

可能的归位错误和可操作项：

- 传感器错误
  - 检查日志以获取详细错误
- Eddy I2C STATUS/DATA 错误。
  - 检查松动的接线。
  - 尝试软件 I2C/降低 I2C 速率
- 无效的读取数据
  - 与 I2C 相同

可能的传感器错误和可操作项：
- 频率超出有效硬范围
  - 检查频率配置
  - 硬件故障
- 频率超出有效软范围
  - 检查频率配置
- 转换看门狗超时
  - 硬件故障

振幅低/高警告消息可能意味着：
- 传感器靠近床
- 传感器远离床
- 温度高于当前校准时的温度
- 缺少电容器

在某些传感器上，无法完全避免振幅警告指示器。

您可以尝试在工作温度下重做 `LDC_CALIBRATE_DRIVE_CURRENT` 校准，或将 `reg_drive_current` 从校准值增加 1-2。

一般来说，它就像发动机检查灯。它可能表示一个问题。