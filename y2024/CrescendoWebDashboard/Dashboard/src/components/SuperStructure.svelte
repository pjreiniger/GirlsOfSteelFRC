<script lang="ts">
  import { getEntry } from "@frc-web-components/svelte";
  let pivotMotorAngle         = getEntry(`/SmartDashboard/super structure/PivotMotorAngle`, false);
  let goalAngle               = getEntry(`/SmartDashboard/super structure/GoalAngle`, false);
  let shooterMotorPercentage  = getEntry(`/SmartDashboard/super structure/ShooterMotorPercentage`, false);
  let pivotMotorPercentage    = getEntry(`/SmartDashboard/super structure/PivotMotorPercentage`, false);
  let hasGamePiece            = getEntry(`/SmartDashboard/super structure/HasGamePiece`, false);
  let intakeMotorPercentage   = getEntry(`/SmartDashboard/super structure/IntakeMotorPercentage`, false);
</script>


<super-structure style="width: 700px; height: 700px;"
    pivotMotorAngle={$pivotMotorAngle}
    goalAngle={$goalAngle}
    shooterMotorPercentage={$shooterMotorPercentage}
    pivotMotorPercentage={$pivotMotorPercentage}
    hasGamePiece={$hasGamePiece}
    intakeMotorPercentage={$intakeMotorPercentage}
/>