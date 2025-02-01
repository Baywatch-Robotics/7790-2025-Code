package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class AprilTagVision extends SubsystemBase{
    
    private Pose2d currentPose;

    PhotonCamera rightCam = new PhotonCamera("RightCam");
    PhotonCamera leftCam = new PhotonCamera("leftCam");
    PhotonCamera limelight = new PhotonCamera("limelight");


public AprilTagVision(){
    
}


}
