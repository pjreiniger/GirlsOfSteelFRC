<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";

  let robotPoseEstimation = getEntry(`/SmartDashboard/Field/Robot`, [0, 0, 0]);
  let predictedPose = getEntry(`/SmartDashboard/Field/futurePosition`, [0, 0, 0]);
  let notes = getEntry(`/SmartDashboard/Field/Notes`, []);
  let aprilTags = getEntry(`/SmartDashboard/Field/AprilTags`, []);
  let camera1DetectedTags = getEntry(`/SmartDashboard/Field/AprilTag1 detected tags`, []);
  let camera2PoseEstimate = getEntry(`/SmartDashboard/Field/AprilTag1 estimated pose`, []);
  let trajectory = getEntry(`/SmartDashboard/Field/Trajectory`, []);
</script>

<frc-field
  crop-left=".1"
  crop-right=".9"
  game="Crescendo"
  rotation-unit="degrees">
  <frc-field-robot
    opacity="0.1"
    pose={$predictedPose} />
  <frc-field-robot
    color="orange"
    pose={$camera2PoseEstimate} />
  <gos-field-image-poses
    image="field-images/notes.png"
    length="0.4"
    opacity="1"
    poses={$notes}
    width="0.4" />
  <gos-field-image-poses
    image="field-images/tag-blue.png"
    length="0.4"
    opacity="1"
    poses={$aprilTags}
    width="0.4" />
  <gos-field-image-poses
    image="field-images/tag-orange.png"
    length="0.4"
    opacity="1"
    poses={$camera1DetectedTags}
    width="0.4" />
  <frc-field-path
    color="green"
    opacity="0.7"
    poses={$trajectory} />

  <frc-field-robot pose={$robotPoseEstimation} />
</frc-field>

<style>
  frc-field {
    width: 900px;
    height: 450px;
  }
</style>