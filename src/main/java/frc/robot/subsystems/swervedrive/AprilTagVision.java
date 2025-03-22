package frc.robot.subsystems.swervedrive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import java.util.Arrays;
import java.util.List;
import java.util.Optional;


public class AprilTagVision extends SubsystemBase {
    
    private static final PhotonCamera rightCam = new PhotonCamera("rightcam");
    private static final PhotonCamera leftCam = new PhotonCamera("leftcam");
    //private static final PhotonCamera limelight = new PhotonCamera("limelight");

    private static final Transform3d rightCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.rightCamXOffset, AprilTagVisionConstants.rightCamYOffset, AprilTagVisionConstants.rightCamZOffset),
            new Rotation3d(AprilTagVisionConstants.rightCamRoll, AprilTagVisionConstants.rightCamPitch, AprilTagVisionConstants.rightCamYaw));

    private static final Transform3d leftCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.leftCamXOffset, AprilTagVisionConstants.leftCamYOffset, AprilTagVisionConstants.leftCamZOffset),
            new Rotation3d(AprilTagVisionConstants.leftCamRoll, AprilTagVisionConstants.leftCamPitch, AprilTagVisionConstants.leftCamYaw));

        /*
    private static final Transform3d limelightToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.limelightXOffset, AprilTagVisionConstants.limelightYOffset, AprilTagVisionConstants.limelightZOffset),
            new Rotation3d(AprilTagVisionConstants.limelightRoll, AprilTagVisionConstants.limelightPitch, AprilTagVisionConstants.limelightYaw));
    */
    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private static PhotonPoseEstimator rightPoseEstimator;
    private static PhotonPoseEstimator leftPoseEstimator;
    //private static PhotonPoseEstimator limelightPoseEstimator;

    // Track pose consistency
    private static int consecutiveValidPoses = 0;
    private static Pose3d lastValidPose = null;

    static {
        rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamToRobot);
        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamToRobot);
        //limelightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, limelightToRobot);

        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //limelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static Optional<EstimatedRobotPose> getRightCamPose(Pose2d prevEstimatedPose) {
        rightPoseEstimator.setReferencePose(prevEstimatedPose);
        return rightPoseEstimator.update(rightCam.getLatestResult());
    }

    
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
    
    /**
     * Check if an estimated pose only uses valid tags from our allowed list
     */
    private static boolean usesOnlyValidTags(EstimatedRobotPose pose) {
        // If no targets were used, consider it invalid
        if (pose.targetsUsed.isEmpty()) {
            return false;
        }
        
        // Determine max distance based on our consistency level
        double maxAllowedDistance = AprilTagVisionConstants.MAX_TAG_DISTANCE;
        
        // If we haven't established a consistent position yet, be more strict
        if (consecutiveValidPoses < AprilTagVisionConstants.REQUIRED_CONSISTENT_POSES) {
            maxAllowedDistance = AprilTagVisionConstants.MAX_TAG_DISTANCE * AprilTagVisionConstants.INITIAL_DISTANCE_MULTIPLIER;
        }
        
        // Check that all targets used are in our valid tag list
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            int tagId = target.getFiducialId();
            
            // Skip if the tag is not in our valid list
            if (!Arrays.asList(AprilTagVisionConstants.VALID_TAG_IDS).contains(tagId)) {
                return false;
            }
            
            // Check if the tag is too far away
            double distanceMeters = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distanceMeters > maxAllowedDistance) {
                // For tags that are far away, we'll be more lenient if we have established a consistent position
                if (consecutiveValidPoses < AprilTagVisionConstants.REQUIRED_CONSISTENT_POSES) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    /**
     * Check if a new pose is consistent with our previous valid pose
     */
    private static boolean isPoseConsistent(Pose3d newPose) {
        // If we don't have a previous pose, this is automatically consistent
        if (lastValidPose == null) {
            return true;
        }
        
        // Check if the new pose is close to our last valid pose
        double positionDelta = lastValidPose.getTranslation().getDistance(newPose.getTranslation());
        double rotationDelta = Math.abs(lastValidPose.getRotation().getZ() - newPose.getRotation().getZ()) 
                              * 180.0 / Math.PI; // convert to degrees
        
        // Adjust thresholds based on consistency level
        double maxJumpDistance = AprilTagVisionConstants.MAX_POSE_JUMP_DISTANCE;
        double maxJumpAngle = AprilTagVisionConstants.MAX_POSE_JUMP_ANGLE_DEGREES;
        
        // Be more lenient if we have a consistent track record
        if (consecutiveValidPoses > AprilTagVisionConstants.REQUIRED_CONSISTENT_POSES) {
            maxJumpDistance *= AprilTagVisionConstants.ESTABLISHED_DISTANCE_MULTIPLIER;
            maxJumpAngle *= AprilTagVisionConstants.ESTABLISHED_DISTANCE_MULTIPLIER;
        }
        
        return positionDelta < maxJumpDistance && rotationDelta < maxJumpAngle;
    }
    
    public static Optional<Pose3d> getBestPoseEstimate(Pose2d prevEstimatedPose) {
        List<Pose3d> validPoses = new ArrayList<>();

        Optional<EstimatedRobotPose> rightCamEstimate = getRightCamPose(prevEstimatedPose);
        if (rightCamEstimate.isPresent()) {
            EstimatedRobotPose estimate = rightCamEstimate.get();
            if (!estimate.targetsUsed.isEmpty() && 
                estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold && 
                usesOnlyValidTags(estimate)) {
                
                validPoses.add(estimate.estimatedPose);
            }
        }   

        Optional<EstimatedRobotPose> leftCamEstimate = getLeftCamPose(prevEstimatedPose);
        if (leftCamEstimate.isPresent()) {
            EstimatedRobotPose estimate = leftCamEstimate.get();
            if (!estimate.targetsUsed.isEmpty() && 
                estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold && 
                usesOnlyValidTags(estimate)) {
                
                validPoses.add(estimate.estimatedPose);
            }
        }
        /*
                Optional<EstimatedRobotPose> limelightEstimate = getLimelightPose(prevEstimatedPose);
        if (limelightEstimate.isPresent()) {
            EstimatedRobotPose estimate = limelightEstimate.get();
            if (estimate.targetsUsed.get(0).getPoseAmbiguity() <= AprilTagVisionConstants.ambiguityThreshold && 
                usesOnlyValidTags(estimate)) {
                validPoses.add(estimate.estimatedPose);
            }
        }
        */
        if (validPoses.isEmpty()) {
            // Reset consistency counter when we have no valid poses
            consecutiveValidPoses = 0;
            return Optional.empty(); // No valid poses found
        }

        Pose3d averagedPose = averagePoses(validPoses);
        
        // Check if the new pose is consistent with previous poses
        if (isPoseConsistent(averagedPose)) {
            consecutiveValidPoses++;
            System.out.println("Consecutive valid poses: " + consecutiveValidPoses);
            lastValidPose = averagedPose;
            return Optional.of(averagedPose);
        } else {
            // If we have a good track record, still use the pose but don't count it as consistent
            if (consecutiveValidPoses >= AprilTagVisionConstants.REQUIRED_CONSISTENT_POSES) {
                System.out.println("Using inconsistent pose due to good history. Delta too large.");
                lastValidPose = averagedPose;
                return Optional.of(averagedPose);
            }
            
            // Otherwise reset our counter and reject the pose
            consecutiveValidPoses = 0;
            return Optional.empty();
        }
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

    /**
     * Reset our consistency tracking - call this when you know the robot has been moved
     * or when you're initializing a new position
     */
    public static void resetPoseConsistency() {
        consecutiveValidPoses = 0;
        lastValidPose = null;
        System.out.println("AprilTag pose consistency tracking reset");
    }
}