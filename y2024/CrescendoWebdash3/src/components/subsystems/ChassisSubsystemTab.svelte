<script lang="ts">
  import GosField from "../GosField.svelte";
</script>

<frc-robot-command
  name="UNINITIALIZED"
  source-key="/Shuffleboard/test commands/Chassis to 45"
  source-provider="NetworkTables"
/>
<frc-robot-command
  source-key="/Shuffleboard/test commands/Chassis to 90"
  source-provider="NetworkTables"
/>
<frc-robot-command
  source-key="/Shuffleboard/test commands/Chassis to -45"
  source-provider="NetworkTables"
/><frc-robot-command
  source-key="/Shuffleboard/test commands/Chassis to -180"
  source-provider="NetworkTables"
/>
<frc-robot-command
  source-key="/Shuffleboard/test commands/Chassis drive to speaker"
  source-provider="NetworkTables"
/>
<frc-robot-command
  source-key="/Shuffleboard/test commands/Chassis drive to amp"
  source-provider="NetworkTables"
/>


<frc-swerve-drivebase
  desiredstates="[0,0,0,0,0,0,0,0]"
  measuredstates="[0,0,0,0,0,0,0,0]"
  sizefrontback="5"
  sizeleftright="4"
  source-key="/SwerveDrive"
  source-provider="NetworkTables"
  wheellocations="[2.5,2,2.5,-2,-2.5,2,-2.5,-2]"
/>

<GosField />