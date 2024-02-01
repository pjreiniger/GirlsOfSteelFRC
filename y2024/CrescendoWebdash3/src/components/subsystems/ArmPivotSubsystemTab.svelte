<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to 0"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to coast"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to 90"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to 45"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to amp angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to ground intake angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to default speaker angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Arm to speaker (from pose)"
      source-provider="NetworkTables" />

  </li>
</ul>
<frc-line-chart
  chart-title="Line Chart"
  source-provider="NetworkTables"
  style="transform-origin: 0 0; transform: translate(50%, -100%);">
  <frc-line-chart-legend source-provider="NetworkTables" />
  <frc-line-chart-axis
    lock-max=""
    lock-min=""
    max="95"
    min="0"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Absolute Encoder Angle"
    source-key="/Arm Pivot Subsystem/Abs Encoder Value"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Relative Encoder Angle"
    source-key="/Arm Pivot Subsystem/Rel Encoder Value"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Goal Angle"
    source-key="/SmartDashboard/super structure/GoalAngle"
    source-provider="NetworkTables" />
</frc-line-chart>

