<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";

  let pose = getEntry(`/SmartDashboard/Field/Robot`, [0, 0, 0]);
  let aprilTags2 = getEntry(`/SmartDashboard/Field/AprilTags`, [1, 2, 3]);
</script>

<frc-field game="Crescendo" crop-left=".1" crop-right=".9" rotation-unit="deg">
  <frc-field-robot pose="{$pose}"></frc-field-robot>
  <frc-field-path color="pink" opacity="1" poses="{$aprilTags2}"></frc-field-path>
  <frc-field-path color="green" opacity="0.7" line-width="10" translations="[1,2,3,4]"></frc-field-path>
</frc-field>

<style>
  frc-field {
    width: 500px;
    height: 300px;
  }
</style>