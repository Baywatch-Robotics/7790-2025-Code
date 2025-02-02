package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagVisionConstants;

public class AprilTagVision extends SubsystemBase{
    
    private Transform3d robotPose;

    PhotonCamera rightCam = new PhotonCamera("RightCam");
    PhotonCamera leftCam = new PhotonCamera("leftCam");
    PhotonCamera limelight = new PhotonCamera("limelight");

    private Transform3d rightCamToRobot = new Transform3d(new Translation3d(AprilTagVisionConstants.rightCamXOffset,AprilTagVisionConstants.rightCamYOffset, AprilTagVisionConstants.rightCamZOffset), new Rotation3d(AprilTagVisionConstants.rightCamRoll, AprilTagVisionConstants.rightCamPitch, AprilTagVisionConstants.rightCamYaw));

    private Transform3d leftCamToRobot = new Transform3d(new Translation3d(AprilTagVisionConstants.leftCamXOffset,AprilTagVisionConstants.leftCamYOffset, AprilTagVisionConstants.leftCamZOffset), new Rotation3d(AprilTagVisionConstants.leftCamRoll, AprilTagVisionConstants.leftCamPitch, AprilTagVisionConstants.leftCamYaw));

    private Transform3d limelightToRobot = new Transform3d(new Translation3d(AprilTagVisionConstants.limelightXOffset,AprilTagVisionConstants.limelightYOffset, AprilTagVisionConstants.limelightZOffset), new Rotation3d(AprilTagVisionConstants.limelightRoll, AprilTagVisionConstants.limelightPitch, AprilTagVisionConstants.limelightYaw));

    
    
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    AprilTagVision(){

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, rightCamToRobot);



        }


}
