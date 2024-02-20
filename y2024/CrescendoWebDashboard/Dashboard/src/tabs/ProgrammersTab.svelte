<script lang="ts">
  import "@frc-web-components/fwc/components";

  import GosField from '../components/GosField.svelte';
  import SuperStructure from '../components/SuperStructure.svelte';
</script>
<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Teleop: Aim at speaker (current pose)"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Teleop: David Drive"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Auto-Intake Piece"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/SpeakerAimAndShoot Default Values"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Auto-Shoot in Amp"
      source-provider="NetworkTables" />

  </li>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Teleop: Aim at speaker (predicted pose)"
      source-provider="NetworkTables" />

  </li>
</ul>
<SuperStructure />
<GosField />
<frc-boolean-box
  label="Arm At Goal"
  source-key="/Arm Pivot Subsystem/Arm At Goal"
  source-provider="NetworkTables" />

<frc-boolean-box
  label="Shooter At Goal"
  source-key="/Shooter Subsystem/Shooter At Goal"
  source-provider="NetworkTables" />

<frc-boolean-box
  label="Chassis At Goal"
  source-key="/Chassis/At Angle Setpoint"
  source-provider="NetworkTables" />

