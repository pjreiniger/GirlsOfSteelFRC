<script lang="ts">
  import "@frc-web-components/fwc/components";

  import GosField from '../components/GosField.svelte';
</script>
<ul>
  <li>
    <frc-robot-command
      name="UNLINKED"
      style:width="250px"
      style:height="50px"
      source-key="/Shuffleboard/test commands/Teleop: Aim at speaker"
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
      source-key="/Shuffleboard/test commands/Auto-Shoot in Speaker"
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
</ul>


<GosField />

