<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/intake in"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/intake out"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/intake until piece"
      source-provider="NetworkTables" />

  </li>
</ul>
<frc-boolean-box
  false-color="#b4a7a7"
  source-key="/SmartDashboard/super structure/HasGamePiece"
  source-provider="NetworkTables"
  true-color="#ffa200" />

<frc-line-chart
  chart-title="Line Chart"
  source-provider="NetworkTables">
  <frc-line-chart-legend source-provider="NetworkTables" />
  <frc-line-chart-axis source-provider="NetworkTables" />
  <frc-line-chart-axis
    side="right"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="Current (Amps)"
    source-key="/Intake Subsystem/Current (Amps)"
    source-provider="NetworkTables" />
</frc-line-chart>

