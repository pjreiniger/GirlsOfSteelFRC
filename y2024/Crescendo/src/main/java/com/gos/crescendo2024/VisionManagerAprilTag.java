package com.gos.crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionManagerAprilTag {

    private static class CameraHelper {
        private final PhotonCamera m_camera;
        private final PhotonPoseEstimator m_poseEstimator;
        private final PhotonCameraSim m_cameraSim;
        private final GoSField24.CameraObject m_fieldObject;
        private final  Transform3d m_robotToCamera;

        private CameraHelper(GoSField24 field, String name, Transform3d robotToCamera) {
            m_robotToCamera = robotToCamera;
            m_camera = new PhotonCamera(name);
            m_poseEstimator =
                new PhotonPoseEstimator(FieldConstants.TAG_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCamera);
            m_fieldObject = new GoSField24.CameraObject(robotToCamera, field, name);

            if (RobotBase.isSimulation()) {
                m_cameraSim = new PhotonCameraSim(m_camera, SimCameraProperties.PERFECT_90DEG());

                m_cameraSim.enableRawStream(true);
                m_cameraSim.enableProcessedStream(true);
                m_cameraSim.enableDrawWireframe(true);
            } else {
                m_cameraSim = null;
            }
        }
    }

    private static final String LEFT_FRONT_CAMERA_NAME = "LeftFront";
    private static final String RIGHT_FRONT_CAMERA_NAME = "RightFront";

    private static final Transform3d LEFT_FRONT_CAMERA_ROBOT_TO_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(12), Units.inchesToMeters(0)),
        new Rotation3d(0, 0, Math.toRadians(-30)));
    private static final Transform3d RIGHT_FRONT_CAMERA_ROBOT_TO_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(0, 0, Math.toRadians(30)));

    private final List<CameraHelper> m_cameras;

    private final VisionSystemSim m_visionSim;

    public VisionManagerAprilTag(GoSField24 field) {
        m_cameras = new ArrayList<>();
        m_cameras.add(new CameraHelper(field, LEFT_FRONT_CAMERA_NAME, LEFT_FRONT_CAMERA_ROBOT_TO_CAM));
        m_cameras.add(new CameraHelper(field, RIGHT_FRONT_CAMERA_NAME, RIGHT_FRONT_CAMERA_ROBOT_TO_CAM));

        if (RobotBase.isSimulation()) {
            m_visionSim = new VisionSystemSim("vision");
            m_visionSim.addAprilTags(FieldConstants.TAG_LAYOUT);

            for (CameraHelper helper : m_cameras) {
                m_visionSim.addCamera(helper.m_cameraSim, helper.m_robotToCamera);
            }
        } else {
            m_visionSim = null;
        }
    }

    public List<EstimatedRobotPose> update(Pose2d robotPose) {
        if (Robot.isSimulation()) {
            m_visionSim.update(robotPose);
        }

        List<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
        for (CameraHelper helper : m_cameras) {
            Optional<EstimatedRobotPose> maybeResult = helper.m_poseEstimator.update();
            helper.m_fieldObject.setEstimate(maybeResult);
            if (maybeResult.isPresent()) {
                estimatedPoses.add(maybeResult.get());
            }
        }

        return estimatedPoses;
    }
}
