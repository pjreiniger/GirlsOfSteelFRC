<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";

  let robotPoseEstimation = getEntry(`/SmartDashboard/Field/Robot`, [0, 0, 0]);
  let notes = getEntry(`/SmartDashboard/Field/Notes`, []);
  let aprilTags = getEntry(`/SmartDashboard/Field/AprilTags`, []);
  let trajectory = getEntry(`/SmartDashboard/Field/Trajectory`, []);
</script>

<frc-field
  crop-left=".1"
  crop-right=".9"
  game="Crescendo"
  rotation-unit="radians">
  <frc-field-robot pose={$robotPoseEstimation} />
  <frc-field-path
    color="orange"
    opacity="1"
    poses={$notes} />
  <frc-field-path
    color="pink"
    opacity="1"
    poses={$aprilTags} />
  <frc-field-path
    color="green"
    opacity="0.7"
    poses={$trajectory} />
</frc-field>

<style>
  frc-field {
    width: 500px;
    height: 300px;
  }
</style>