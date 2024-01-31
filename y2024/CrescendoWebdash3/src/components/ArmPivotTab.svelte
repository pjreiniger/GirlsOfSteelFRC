<frc-line-chart
        chart-title="Line Chart"
        source-provider="NetworkTables"
        style="transform: translate(240px, 120px);"
    >
        <frc-line-chart-legend source-provider="NetworkTables"></frc-line-chart-legend>
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
        ></frc-line-chart-data></frc-line-chart><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(11px, 15px);"
        source-key="/Shuffleboard/test commands/Arm to 0"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(8px, 184px);"
        source-key="/Shuffleboard/test commands/Arm to coast"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(10px, 124px);"
        source-key="/Shuffleboard/test commands/Arm to 90"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(11px, 71px);"
        source-key="/Shuffleboard/test commands/Arm to 45"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(4px, 269px); width: 181px; height: 50px;"
        source-key="/Shuffleboard/test commands/Arm to amp angle"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(14px, 333px); width: 170px; height: 50px;"
        source-key="/Shuffleboard/test commands/Arm to ground intake angle"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(7px, 392px); width: 176px; height: 50px;"
        source-key="/Shuffleboard/test commands/Arm to default speaker angle"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        style="transform: translate(6px, 451px); width: 178px; height: 50px;"
        source-key="/Shuffleboard/test commands/Arm to speaker (from pose)"
    ></frc-robot-command><vaadin-number-field
        label="Rel Arm Angle"
        value="0"
        has-value=""
        has-label=""
        source-provider="NetworkTables"
        style="transform: translate(272px, 609px);"
        source-key="/Arm Pivot Subsystem/Rel Encoder Value"
        placeholder=""
        min="NaN"
        max="NaN"
    ><input
            slot="input"
            value="0"
            type="number"
            id="vaadin-number-field-627"
            aria-labelledby="label-vaadin-number-field-632"
            aria-describedby=""
            min="undefined"
            max="undefined"
            step="any"
        ><label
            slot="label"
            id="label-vaadin-number-field-632"
            for="vaadin-number-field-627"
        >Rel Arm Angle</label>
        <div
            slot="error-message"
            id="error-message-vaadin-number-field-416"
            hidden=""
        ></div>
    </vaadin-number-field><vaadin-number-field
        label="Percent Output"
        value="0"
        has-value=""
        has-label=""
        source-provider="NetworkTables"
        style="transform: translate(423px, 608px);"
        source-key="/SmartDashboard/super structure/PivotMotorPercentage"
        placeholder=""
        min="NaN"
        max="NaN"
    ><input
            slot="input"
            value="0"
            type="number"
            id="vaadin-number-field-628"
            aria-labelledby="label-vaadin-number-field-633"
            aria-describedby=""
            min="undefined"
            max="undefined"
            step="any"
        ><label
            slot="label"
            id="label-vaadin-number-field-633"
            for="vaadin-number-field-628"
        >Percent Output</label>
        <div
            slot="error-message"
            id="error-message-vaadin-number-field-417"
            hidden=""
        ></div>
    </vaadin-number-field><vaadin-number-field
        label="Abs Arm ANgle"
        value="0"
        has-value=""
        has-label=""
        source-provider="NetworkTables"
        style="transform: translate(272px, 700px);"
        source-key="/Arm Pivot Subsystem/Abs Encoder Value"
        placeholder=""
        min="NaN"
        max="NaN"
    ><input
            slot="input"
            value="0"
            type="number"
            id="vaadin-number-field-684"
            aria-labelledby="label-vaadin-number-field-689"
            aria-describedby=""
            min="undefined"
            max="undefined"
            step="any"
        ><label
            slot="label"
            id="label-vaadin-number-field-689"
            for="vaadin-number-field-684"
        >Abs Arm ANgle</label>
        <div
            slot="error-message"
            id="error-message-vaadin-number-field-453"
            hidden=""
        ></div>
    </vaadin-number-field>