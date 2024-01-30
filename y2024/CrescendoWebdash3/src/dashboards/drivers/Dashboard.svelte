<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";

  let toggled = getEntry(`/dashboard/toggled`, false);
  let pose = getEntry(`/SmartDashboard/Field/Robot`, [0, 0, 0]);
  let count = getEntry(`/dashboard/count`, 0);

  const updateToggle = () => {
    toggled.setValue(!$toggled);
  };

import GosField from '../../components/GosField.svelte';
</script>

<div>
  <frc-sendable-chooser
    source-key="/Shuffleboard/Autonomous/SendableChooser[0]"
  />
  <frc-basic-fms-info source-key="/FMSInfo" />
  <frc-toggle-button
    label="Toggle Button"
    toggled={$toggled}
    on:toggle={updateToggle}
  />
</div>
<GosField />
