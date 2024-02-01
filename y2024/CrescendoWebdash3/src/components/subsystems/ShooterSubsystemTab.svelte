<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Shooter Percent Output"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/shooter RPM: default"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Shooter RPM: 3500"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      source-key="/Shuffleboard/test commands/Shooter RPM: 4000"
      source-provider="NetworkTables" />

  </li>
</ul>
<frc-line-chart
  chart-title="Line Chart"
  source-provider="NetworkTables"
  style="transform-origin: 0 0; transform: translate(50%, -50%);">
  <frc-line-chart-legend source-provider="NetworkTables" />
  <frc-line-chart-axis
    lock-max=""
    lock-min=""
    max="4000"
    source-provider="NetworkTables" />
  <frc-line-chart-data
    display-name="RPM"
    source-key="/Shooter Subsystem/Velocity (RPM)"
    source-provider="NetworkTables" />
</frc-line-chart>

