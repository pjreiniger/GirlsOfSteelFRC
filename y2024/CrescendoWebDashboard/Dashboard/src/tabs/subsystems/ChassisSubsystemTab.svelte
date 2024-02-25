<script lang="ts">
  import "@frc-web-components/fwc/components";

  import GosField from '../../components/GosField.svelte';
</script>
<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis to 45"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis to 90"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis to -45"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis to -180"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis drive to speaker"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis drive to amp"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis Set Pose Subwoofer Bottom"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis Set Pose Subwoofer Mid"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Chassis Set Pose Subwoofer Top"
      source-provider="NetworkTables" />

  </li>
</ul>
<frc-swerve-drivebase
  desiredstates="[0,0,0,0,0,0,0,0]"
  measuredstates="[0,0,0,0,0,0,0,0]"
  sizefrontback="5"
  sizeleftright="4"
  source-key="/SwerveDrive"
  source-provider="NetworkTables"
  wheellocations="[2.5,2,2.5,-2,-2.5,2,-2.5,-2]" />

<frc-boolean-box
  source-key="/Chassis/At Angle Setpoint"
  source-provider="NetworkTables" />

<GosField />
