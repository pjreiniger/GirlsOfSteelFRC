package com.gos.crescendo2024;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class GoSField {
    @SuppressWarnings("PMD.DataClass")
    public static class CameraObject {
        private final Transform3d m_robotToCamera;
        private final FieldObject2d m_estimatedPosition;
        private final FieldObject2d m_detectedTags;

        public CameraObject(Transform3d robotToCamera, GoSField24 field, String cameraName) {
            m_robotToCamera = robotToCamera;
            m_estimatedPosition = field.m_field.getObject(cameraName + ": EstimatedPosition");
            m_detectedTags = field.m_field.getObject(cameraName + ": DetectedTags");
        }

        public void setEstimate(Optional<EstimatedRobotPose> maybeEstimate) {
            if (maybeEstimate.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = maybeEstimate.get();
                Pose3d pose = estimatedRobotPose.estimatedPose;

                List<Pose2d> estimatedTagPoses = new ArrayList<>();
                for (PhotonTrackedTarget targetUsed : estimatedRobotPose.targetsUsed) {
                    estimatedTagPoses.add(pose
                        .transformBy(m_robotToCamera)
                        .transformBy(targetUsed.getBestCameraToTarget())
                        .toPose2d());
                }
                m_estimatedPosition.setPose(pose.toPose2d());
                m_detectedTags.setPoses(estimatedTagPoses);
            } else {
                m_estimatedPosition.setPoses();
            }
        }
    }

    private final Field2d m_field;
    private final FieldObject2d m_currentTrajectoryObject;
    private final FieldObject2d m_trajectorySetpoint;
    private final FieldObject2d m_odometryObject;
    private final FieldObject2d m_simulatedNotes;
    private final FieldObject2d m_detectedNotePoses;

    public GoSField() {
        m_field = new Field2d();
        m_currentTrajectoryObject = m_field.getObject("Trajectory");
        m_detectedNotePoses = m_field.getObject("Notes");
        m_trajectorySetpoint = m_field.getObject("TrajectoryTargetPose");
        m_odometryObject = m_field.getObject("OldOdometry");
        m_aprilTagObjects = m_field.getObject("AprilTags");
        m_simulatedNotes = m_field.getObject("SimulatedNotes");
        m_detectedNotes = m_field.getObject("DetectedNotes");

        List<Pose2d> tagPoses = new ArrayList<>();
        for (AprilTag tag : FieldConstants.TAG_LAYOUT.getTags()) {
            tagPoses.add(tag.pose.toPose2d());
        }
        FieldObject2d aprilTagObjects = m_field.getObject("AprilTags"); // NOPMD(CloseResource)
        aprilTagObjects.setPoses(tagPoses);
    }

    public void setTrajectory(List<Pose2d> trajectory) {
        m_currentTrajectoryObject.setPoses(trajectory);
    }

    public void setTrajectorySetpoint(Pose2d pose) {
        m_trajectorySetpoint.setPose(pose);
    }

    public void setOdometry(Pose2d pose) {
        m_odometryObject.setPose(pose);
    }

    public void setPoseEstimate(Pose2d pose) {
        m_field.setRobotPose(pose);
    }

    public Sendable getSendable() {
        return m_field;
    }

    public void drawNotePoses(List<Pose2d> poses) {
        m_detectedNotePoses.setPoses(poses);
    }


    public void addSimulatedNotes(List<Pose2d> notePoses) {
        m_simulatedNotes.setPoses(notePoses);
    }

    public void addDetectedNotes(List<Pose2d> notePoses) {
        m_detectedNotes.setPoses(notePoses);
    }

}
