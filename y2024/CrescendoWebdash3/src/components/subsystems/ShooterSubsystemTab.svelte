
<ul>

<li>
<frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Shooter Percent Output"
    ></frc-robot-command>
</li>

<li>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/shooter RPM: default"
    ></frc-robot-command>
</li>

<li>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Shooter RPM: 3500"
    ></frc-robot-command>
</li>

<li>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Shooter RPM: 4000"
    ></frc-robot-command>
</li>
</ul>

<frc-line-chart
        chart-title="Line Chart"
        source-provider="NetworkTables"
        style="transform-origin: 0 0; transform: translate(50%, -50%);"
    >
        <frc-line-chart-legend source-provider="NetworkTables"></frc-line-chart-legend>
        <frc-line-chart-axis
            source-provider="NetworkTables"
            max="4000"
            lock-min=""
            lock-max=""
        ></frc-line-chart-axis>

        <frc-line-chart-data
            source-provider="NetworkTables"
            source-key="/Shooter Subsystem/Velocity (RPM)"
            display-name="RPM"
        ></frc-line-chart-data>
    </frc-line-chart>