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
    
    private static final PhotonCamera rightCam = new PhotonCamera("rightcam");
    //private static final PhotonCamera leftCam = new PhotonCamera("leftcam");
    //private static final PhotonCamera limelight = new PhotonCamera("limelight");

    private static final Transform3d rightCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.rightCamXOffset, AprilTagVisionConstants.rightCamYOffset, AprilTagVisionConstants.rightCamZOffset),
            new Rotation3d(AprilTagVisionConstants.rightCamRoll, AprilTagVisionConstants.rightCamPitch, AprilTagVisionConstants.rightCamYaw));

    private static final Transform3d leftCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.leftCamXOffset, AprilTagVisionConstants.leftCamYOffset, AprilTagVisionConstants.leftCamZOffset),
            new Rotation3d(AprilTagVisionConstants.leftCamRoll, AprilTagVisionConstants.leftCamPitch, AprilTagVisionConstants.leftCamYaw));

    private static final Transform3d limelightToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.limelightXOffset, AprilTagVisionConstants.limelightYOffset, AprilTagVisionConstants.limelightZOffset),
            new Rotation3d(AprilTagVisionConstants.limelightRoll, AprilTagVisionConstants.limelightPitch, AprilTagVisionConstants.limelightYaw));

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private static PhotonPoseEstimator rightPoseEstimator;
    private static PhotonPoseEstimator leftPoseEstimator;
    private static PhotonPoseEstimator limelightPoseEstimator;



    static {
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

    /*
    public static Optional<EstimatedRobotPose> getLeftCamPose(Pose2d prevEstimatedPose) {
        leftPoseEstimator.setReferencePose(prevEstimatedPose);
        return leftPoseEstimator.update(leftCam.getLatestResult());
    }

    /*
    public static Optional<EstimatedRobotPose> getLimelightPose(Pose2d prevEstimatedPose) {
        limelightPoseEstimator.setReferencePose(prevEstimatedPose);
        return limelightPoseEstimator.update(limelight.getLatestResult());
    }
    */
    
    public static Optional<Pose3d> getBestPoseEstimate(Pose2d prevEstimatedPose) {
        List<Pose3d> validPoses = new ArrayList<>();

        Optional<EstimatedRobotPose> rightCamEstimate = getRightCamPose(prevEstimatedPose);
        if (rightCamEstimate.isPresent()) {
            EstimatedRobotPose estimate = rightCamEstimate.get();
            if (estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold) {
                validPoses.add(estimate.estimatedPose);
            }
        }
/*
        Optional<EstimatedRobotPose> leftCamEstimate = getLeftCamPose(prevEstimatedPose);
        if (leftCamEstimate.isPresent()) {
            EstimatedRobotPose estimate = leftCamEstimate.get();
            if (estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold) {
                validPoses.add(estimate.estimatedPose);
            }
        }
        /*
                Optional<EstimatedRobotPose> limelightEstimate = getLimelightPose(prevEstimatedPose);
        if (limelightEstimate.isPresent()) {
            EstimatedRobotPose estimate = limelightEstimate.get();
            if (estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold) {
                validPoses.add(estimate.estimatedPose);
            }
        }
        */
        if (validPoses.isEmpty()) {
            return Optional.empty(); // No valid poses found
        }

        return Optional.of(averagePoses(validPoses));
    }

    private static Pose3d averagePoses(List<Pose3d> poses) {
        // If only one pose, return it directly
        if (poses.size() == 1) {
            return poses.get(0);
        }
        
        // Average the positions
        double x = 0, y = 0, z = 0;
        
        // We'll use quaternions to average rotations correctly
        double qw = 0, qx = 0, qy = 0, qz = 0;
        
        for (Pose3d pose : poses) {
            // Add position components
            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            
            // Convert rotation to quaternion and add components
            // Note: WPILib Rotation3d stores quaternions internally
            qw += pose.getRotation().getQuaternion().getW();
            qx += pose.getRotation().getQuaternion().getX();
            qy += pose.getRotation().getQuaternion().getY();
            qz += pose.getRotation().getQuaternion().getZ();
        }
        
        int count = poses.size();
        
        // Average the position
        Translation3d avgTranslation = new Translation3d(x / count, y / count, z / count);
        
        // Average and normalize the quaternion
        double length = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        qw /= length;
        qx /= length;
        qy /= length;
        qz /= length;
        
        // Create the rotation from the averaged quaternion
        Rotation3d avgRotation = new Rotation3d(
            new edu.wpi.first.math.geometry.Quaternion(qw, qx, qy, qz));
        
        return new Pose3d(avgTranslation, avgRotation);
    }
}