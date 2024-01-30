<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";
  import MyElement from "./MyElement.svelte";

  let toggled = getEntry(`/dashboard/toggled`, false);
  let pose = getEntry(`/SmartDashboard/Field/Robot`, [0, 0, 0]);
  let count = getEntry(`/dashboard/count`, 0);

  const updateToggle = () => {
    toggled.setValue(!$toggled);
  };
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
  <MyElement count={$count} onIncrement={(value) => count.setValue(value)} />
</div>
<frc-field>
      <frc-field-path color="green" opacity="0.7" line-width="10" translations="[7,5,9,5,9,3,7,3,7,5,8,7,9,5]"></frc-field-path>
    </frc-field>

<style>
  div {
    display: flex;
    gap: 15px;
    align-items: start;
  }

  frc-field {
    width: 500px;
    height: 300px;
  }
</style>
