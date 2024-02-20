<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to 0"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to coast"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to 90"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to 45"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to amp angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to ground intake angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to default speaker angle"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm to speaker (from pose)"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Arm Resync Encoder"
      source-provider="NetworkTables" />

  </li>
</ul>
<frc-line-chart
  style:transform-origin="0 0"
  style:transform="translate(50%, -100%)"
  chart-title="Line Chart"
  source-provider="NetworkTables">
  <frc-line-chart-legend source-provider="NetworkTables" />
  <frc-line-chart-axis
    lock-max=""
    lock-min=""
    max="95"
    min="0"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Absolute Encoder Angle"
    source-key="/Arm Pivot Subsystem/Abs Encoder Position"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Relative Encoder Angle"
    source-key="/Arm Pivot Subsystem/Rel Encoder Position"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Goal Angle"
    source-key="/SmartDashboard/super structure/GoalAngle"
    source-provider="NetworkTables" />
</frc-line-chart>

<frc-line-chart
  style:transform-origin="0 0"
  style:transform="translate(50%, -100%)"
  chart-title="Line Chart"
  source-provider="NetworkTables">
  <frc-line-chart-legend source-provider="NetworkTables" />
  <frc-line-chart-axis
    max="50"
    min="-50"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Abs  Velocity"
    source-key="/Arm Pivot Subsystem/Abs Encoder Velocity"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Rel Velocity"
    source-key="/Arm Pivot Subsystem/Rel Encoder Velocity"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Setpoint Velocity"
    source-key="/Arm Pivot Subsystem/Setpoint Velocity"
    source-provider="NetworkTables" />
</frc-line-chart>

<frc-boolean-box
  label="Arm At Goal"
  source-key="/Arm Pivot Subsystem/Arm At Goal"
  source-provider="NetworkTables" />

