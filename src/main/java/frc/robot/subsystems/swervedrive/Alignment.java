package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Alignment extends SubsystemBase {

    private PhotonCamera leftCam = new PhotonCamera("leftCam");
    private PhotonCamera rightCam = new PhotonCamera("rightCam");

    public static double averageSideDistance = 0;
    public static double averageNormalDistance = 0;
    public static double averageHeading = 0;

    private double P = ReefAlignmentConstants.P;

    public double calculateReefAlignment() {
        // Get the data from the left camera
        var leftResult = leftCam.getLatestResult();
        // Get the data from the right camera
        var rightResult = rightCam.getLatestResult();

        // Check if both cameras have targets
        if (leftResult.hasTargets() && rightResult.hasTargets()) {

            double leftSideDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double leftNormalDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double leftHeading = PhotonUtils.getYawToPose();

            double rightSideDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double rightNormalDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double rightHeading = PhotonUtils.calculateAngleToTargetDegrees();

            // Average the values from both cameras
            averageSideDistance = (leftSideDistance + rightSideDistance) / 2;
            averageNormalDistance = (leftNormalDistance + rightNormalDistance) / 2;
            averageHeading = (leftHeading + rightHeading) / 2;

        } else if (leftResult.hasTargets()) {
            // If only left camera has targets, use left camera values
            double leftSideDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double leftNormalDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double leftHeading = PhotonUtils.calculateAngleToTargetDegrees();

            // Use left camera results
            averageSideDistance = leftSideDistance;
            averageNormalDistance = leftNormalDistance;
            averageHeading = leftHeading;

        } else if (rightResult.hasTargets()) {
            // If only right camera has targets, use right camera values
            double rightSideDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double rightNormalDistance = PhotonUtils.calculateDistanceToTargetMeters();
            double rightHeading = PhotonUtils.calculateDistanceToTargetMeters();

            // Use right camera results
            averageSideDistance = rightSideDistance;
            averageNormalDistance = rightNormalDistance;
            averageHeading = rightHeading;

            
        }
        }
    }
