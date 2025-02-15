package frc.robot.subsystems.swervedrive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagVisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagVision extends SubsystemBase {
    
        private final static PhotonCamera rightCam = new PhotonCamera("RightCam");
        private final static PhotonCamera leftCam = new PhotonCamera("LeftCam");
        private final static PhotonCamera limelight = new PhotonCamera("Limelight");
    
        private final Transform3d rightCamToRobot = new Transform3d(
                new Translation3d(AprilTagVisionConstants.rightCamXOffset, AprilTagVisionConstants.rightCamYOffset, AprilTagVisionConstants.rightCamZOffset),
                new Rotation3d(AprilTagVisionConstants.rightCamRoll, AprilTagVisionConstants.rightCamPitch, AprilTagVisionConstants.rightCamYaw));
    
        private final Transform3d leftCamToRobot = new Transform3d(
                new Translation3d(AprilTagVisionConstants.leftCamXOffset, AprilTagVisionConstants.leftCamYOffset, AprilTagVisionConstants.leftCamZOffset),
                new Rotation3d(AprilTagVisionConstants.leftCamRoll, AprilTagVisionConstants.leftCamPitch, AprilTagVisionConstants.leftCamYaw));
    
        private final Transform3d limelightToRobot = new Transform3d(
                new Translation3d(AprilTagVisionConstants.limelightXOffset, AprilTagVisionConstants.limelightYOffset, AprilTagVisionConstants.limelightZOffset),
                new Rotation3d(AprilTagVisionConstants.limelightRoll, AprilTagVisionConstants.limelightPitch, AprilTagVisionConstants.limelightYaw));
    
        private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
        public static PhotonPoseEstimator rightPoseEstimator;
            private static PhotonPoseEstimator leftPoseEstimator;
            private static PhotonPoseEstimator limelightPoseEstimator;
            
                public AprilTagVision() {
        
                    rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamToRobot);
                    leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamToRobot);
                    limelightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, limelightToRobot);
            
            
                    rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                    leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                    limelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
            
            
                public static Optional<EstimatedRobotPose> getRightCamPose(Pose2d prevEstimatedPose) {
                rightPoseEstimator.setReferencePose(prevEstimatedPose);
            return rightPoseEstimator.update(rightCam.getLatestResult());
    }
    
        public static Optional<EstimatedRobotPose> getLeftCamPose(Pose2d prevEstimatedPose) {
        leftPoseEstimator.setReferencePose(prevEstimatedPose);
        return leftPoseEstimator.update(leftCam.getLatestResult());
}

    public static Optional<EstimatedRobotPose> getLimelightPose(Pose2d prevEstimatedPose) {
    limelightPoseEstimator.setReferencePose(prevEstimatedPose);
    return limelightPoseEstimator.update(limelight.getLatestResult());
}


 /**
     * Gets the best possible estimated pose by averaging valid camera poses.
     */
    public static Optional<Pose3d> getBestPoseEstimate(Pose2d prevEstimatedPose) {
        List<Pose3d> validPoses = new ArrayList<>();

        getRightCamPose(prevEstimatedPose).ifPresent(estimate -> validPoses.add(estimate.estimatedPose));
        getLeftCamPose(prevEstimatedPose).ifPresent(estimate -> validPoses.add(estimate.estimatedPose));
        getLimelightPose(prevEstimatedPose).ifPresent(estimate -> validPoses.add(estimate.estimatedPose));

        if (validPoses.isEmpty()) {
            return Optional.empty(); // No valid poses
        }

        return Optional.of(averagePoses(validPoses));
    }

    /**
     * Averages multiple Pose3d estimates.
     */
    private static Pose3d averagePoses(List<Pose3d> poses) {
        double x = 0, y = 0, z = 0;
        double roll = 0, pitch = 0, yaw = 0;
        int count = poses.size();

        for (Pose3d pose : poses) {
            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            roll += pose.getRotation().getX();
            pitch += pose.getRotation().getY();
            yaw += pose.getRotation().getZ();
        }

        return new Pose3d(
            x / count, y / count, z / count, 
            new Rotation3d(roll / count, pitch / count, yaw / count)
        );
    }
}