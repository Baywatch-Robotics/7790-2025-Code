package frc.robot.subsystems.swervedrive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagVisionConstants;
import swervelib.SwerveDrive;

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

    private static Pose2d previousRightCam = new Pose2d();
    private static Pose2d previousLeftCam = new Pose2d();

    private static boolean isSwerveReset = false; // Flag to check if swerve has been reset

    // Track pose consistency
    private static int consecutiveValidPoses = 0;

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
        Optional<EstimatedRobotPose> newPos = rightPoseEstimator.update(rightCam.getLatestResult());
        previousRightCam = newPos.get().estimatedPose.toPose2d();
        return newPos;
    }

    
    public static Optional<EstimatedRobotPose> getLeftCamPose(Pose2d prevEstimatedPose) {
        leftPoseEstimator.setReferencePose(prevEstimatedPose);
        Optional<EstimatedRobotPose> newPos =  leftPoseEstimator.update(leftCam.getLatestResult());
        previousLeftCam = newPos.get().estimatedPose.toPose2d();
        return newPos;
    }

    public static void updateSwerve(SwerveDrive swerve)
    {
        EstimatedRobotPose rightCamPose = getRightCamPose(previousRightCam).orElse(null);
        EstimatedRobotPose leftCamPose = getLeftCamPose(previousLeftCam).orElse(null);

        if(!isSwerveReset)
        {
            swerve.resetOdometry(leftCamPose.estimatedPose.toPose2d());
            isSwerveReset = true;
        }

        Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);        

        swerve.addVisionMeasurement(leftCamPose.estimatedPose.toPose2d() , leftCamPose.timestampSeconds,kMultiTagStdDevs );
        swerve.addVisionMeasurement(rightCamPose.estimatedPose.toPose2d() , rightCamPose.timestampSeconds,kMultiTagStdDevs );

       
        
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
    
}