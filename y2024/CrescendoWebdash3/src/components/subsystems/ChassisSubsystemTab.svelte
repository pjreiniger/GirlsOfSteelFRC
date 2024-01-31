<script lang="ts">
  import GosField from "../GosField.svelte";
</script>

<frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis to 45"
        name="UNINITIALIZED"
    ></frc-robot-command>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis to 90"
    ></frc-robot-command>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis to -45"
    ></frc-robot-command><frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis to -180"
    ></frc-robot-command>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis drive to speaker"
    ></frc-robot-command>
    <frc-robot-command
        source-provider="NetworkTables"
        source-key="/Shuffleboard/test commands/Chassis drive to amp"
    ></frc-robot-command>


    <frc-swerve-drivebase
        source-provider="NetworkTables"
        source-key="/SwerveDrive"
        wheellocations="[2.5,2,2.5,-2,-2.5,2,-2.5,-2]"
        measuredstates="[0,0,0,0,0,0,0,0]"
        desiredstates="[0,0,0,0,0,0,0,0]"
        sizeleftright="4"
        sizefrontback="5"
    ></frc-swerve-drivebase>

    <GosField />