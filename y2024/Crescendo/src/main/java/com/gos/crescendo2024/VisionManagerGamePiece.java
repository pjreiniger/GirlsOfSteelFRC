package com.gos.crescendo2024;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionManagerGamePiece {
    private static final Transform3d CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(10), Units.inchesToMeters(15)),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
        )
    );
    private static final GosDoubleProperty SIMULATED_NOTE_X = new GosDoubleProperty(false, "SimulatedNoteX", 1);
    private static final GosDoubleProperty SIMULATED_NOTE_Y = new GosDoubleProperty(false, "SimulatedNoteY", 2);

    private final PhotonCamera m_camera;

    private final VisionTargetSim m_simulatedNote;
    private final List<VisionTargetSim> m_defaultSimulatedNotes;

    private final GoSField24 m_field;


    private final VisionSystemSim m_visionSim;

    public VisionManagerGamePiece(GoSField24 field) {
        m_field = field;
        m_camera = new PhotonCamera("GamePieceVision");

        if (RobotBase.isSimulation()) {
            PhotonCameraSim cameraSim = new PhotonCameraSim(m_camera, SimCameraProperties.PERFECT_90DEG());

            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);

            double SPIKE_X = Units.feetToMeters(5);

            TargetModel TARGET_MODEL = new TargetModel(Units.inchesToMeters(18));

            m_simulatedNote = new VisionTargetSim(new Pose3d(), TARGET_MODEL);

            m_defaultSimulatedNotes = new ArrayList<>();
            m_defaultSimulatedNotes.add(m_simulatedNote);

            for (Translation2d stagingLocation : FieldConstants.StagingLocations.CENTER_LINE_TRANSLATIONS) {
                m_defaultSimulatedNotes.add(new VisionTargetSim(new Pose3d(new Pose2d(stagingLocation, new Rotation2d())), TARGET_MODEL));
            }

            for (Translation2d stagingLocation : FieldConstants.StagingLocations.SPIKE_TRANSLATIONS) {
                m_defaultSimulatedNotes.add(new VisionTargetSim(new Pose3d(new Pose2d(stagingLocation, new Rotation2d())), TARGET_MODEL));
            }

            m_visionSim = new VisionSystemSim("GamePieceSim");
            m_visionSim.addCamera(cameraSim, CAMERA_TRANSFORM);

            for (VisionTargetSim targetSim : m_defaultSimulatedNotes) {
                m_visionSim.addVisionTargets("Piece", targetSim);
            }
        } else {
            m_simulatedNote = null;
            m_visionSim = null;
            m_defaultSimulatedNotes = null;
        }
    }

    public void update(Pose2d robotPose) {
        if (Robot.isSimulation()) {
            m_visionSim.update(robotPose);
            m_simulatedNote.setPose(new Pose3d(
                SIMULATED_NOTE_X.getValue(),
                SIMULATED_NOTE_Y.getValue(),
                0,
                new Rotation3d()
            ));

            List<Pose2d> notePoses = new ArrayList<>();
            for (VisionTargetSim targetSim : m_defaultSimulatedNotes) {
                notePoses.add(targetSim.getPose().toPose2d());
            }
            m_field.addSimulatedNotes(notePoses);
        }

        PhotonPipelineResult latestResult = m_camera.getLatestResult();
        List<Pose2d> notePoses = new ArrayList<>();
        for (PhotonTrackedTarget target : latestResult.targets) {
            notePoses.add(getPoseFromTarget(robotPose, target));
        }
        m_field.addDetectedNotes(notePoses);
    }

    private Pose2d getPoseFromTarget(Pose2d robotPose, PhotonTrackedTarget target) {
//        double distanceMeters =
//            calculateDistanceToTargetMeters(
//                CAMERA_TRANSFORM.getTranslation().getZ(),
//                0,
//                -CAMERA_TRANSFORM.getRotation().getY(),
//                Units.degreesToRadians(target.getPitch()),
//                Units.degreesToRadians(target.getYaw()));

//        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
//            distanceMeters, Rotation2d.fromDegrees(target.getYaw() + Math.toDegrees(CAMERA_TRANSFORM.getRotation().getZ())));
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
            PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_TRANSFORM.getTranslation().getZ(),
                0,
                -CAMERA_TRANSFORM.getRotation().getY(),
                Units.degreesToRadians(target.getPitch())),
            Rotation2d.fromDegrees(target.getYaw()));
        translation = new Translation2d(translation.getX() + CAMERA_TRANSFORM.getX(), translation.getY() + CAMERA_TRANSFORM.getY());

        Transform2d transform = new Transform2d(translation, new Rotation2d());
        return robotPose.transformBy(transform);
    }

//    private static double calculateDistanceToTargetMeters(double cameraHeightMeters, double targetHeightMeters, double cameraPitchRadians, double targetPitchRadians, double targetYawRadians) {
//        return (targetHeightMeters - cameraHeightMeters)
//            / (Math.tan(cameraPitchRadians + targetPitchRadians) * Math.cos(targetYawRadians));
//    }

    public Optional<Pose2d> getGamePiecePose(Pose2d robotPose) {
        PhotonPipelineResult latestResult = m_camera.getLatestResult();
        if (latestResult.hasTargets()) {
            return Optional.of(getPoseFromTarget(robotPose, latestResult.getBestTarget()));
        }
        return Optional.empty();
    }
}
