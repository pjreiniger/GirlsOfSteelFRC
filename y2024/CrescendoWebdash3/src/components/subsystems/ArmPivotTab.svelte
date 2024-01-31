
<ul>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to coast"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to 90"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to 45"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to amp angle"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to ground intake angle"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to default speaker angle"
      source-provider="NetworkTables" />
  </li>

  <li>
    <frc-robot-command
      source-key="/Shuffleboard/test commands/Arm to speaker (from pose)"
      source-provider="NetworkTables" />
  </li>

</ul>

<frc-line-chart
  chart-title="Line Chart"
  source-provider="NetworkTables"
        style="transform-origin: 0 0; transform: translate(50%, -100%);"
>
  <frc-line-chart-legend source-provider="NetworkTables"/>
  <frc-line-chart-axis
    source-provider="NetworkTables"
    min="0"
    max="95"
    lock-min=""
    lock-max=""
  ></frc-line-chart-axis>

  <frc-line-chart-data
    source-provider="NetworkTables"
    display-name="Absolute Encoder Angle"
    source-key="/Arm Pivot Subsystem/Abs Encoder Value"
  ></frc-line-chart-data>
  <frc-line-chart-data
    source-provider="NetworkTables"
    display-name="Relative Encoder Angle"
    source-key="/Arm Pivot Subsystem/Rel Encoder Value"
  ></frc-line-chart-data><frc-line-chart-data
    source-provider="NetworkTables"
    display-name="Goal Angle"
    source-key="/SmartDashboard/super structure/GoalAngle"
  ></frc-line-chart-data></frc-line-chart>