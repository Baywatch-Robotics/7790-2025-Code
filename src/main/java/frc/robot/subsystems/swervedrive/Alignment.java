import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Alignment extends SubsystemBase {

    private PhotonCamera leftCam = new PhotonCamera("leftCam");
    private PhotonCamera rightCam = new PhotonCamera("rightCam");

    private double averageSideDistance = 0;
    private double averageNormalDistance = 0;
    private double averageHeading = 0;

    private double P = ReefAlignmentConstants.P;

    public void calculateReefAlignment(int tagId) {
        // Get the data from the left camera
        var leftResult = leftCam.getLatestResult();
        // Get the data from the right camera
        var rightResult = rightCam.getLatestResult();

        // Check if both cameras have targets
        if (leftResult.hasTargets() && rightResult.hasTargets()) {
            // Calculate side distance, normal distance, and heading from left camera
            double leftSideDistance = PhotonUtils.calculateDistanceToTargetMeters(leftResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double leftNormalDistance = PhotonUtils.calculateDistanceToTargetMeters(leftResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double leftHeading = PhotonUtils.calculateAngleToTargetDegrees(leftResult);

            // Calculate side distance, normal distance, and heading from right camera
            double rightSideDistance = PhotonUtils.calculateDistanceToTargetMeters(rightResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double rightNormalDistance = PhotonUtils.calculateDistanceToTargetMeters(rightResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double rightHeading = PhotonUtils.calculateAngleToTargetDegrees(rightResult);

            // Average the values from both cameras
            averageSideDistance = (leftSideDistance + rightSideDistance) / 2;
            averageNormalDistance = (leftNormalDistance + rightNormalDistance) / 2;
            averageHeading = (leftHeading + rightHeading) / 2;

        } else if (leftResult.hasTargets()) {
            // If only left camera has targets, use left camera values
            double leftSideDistance = PhotonUtils.calculateDistanceToTargetMeters(leftResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double leftNormalDistance = PhotonUtils.calculateDistanceToTargetMeters(leftResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double leftHeading = PhotonUtils.calculateAngleToTargetDegrees(leftResult);

            // Use left camera results
            averageSideDistance = leftSideDistance;
            averageNormalDistance = leftNormalDistance;
            averageHeading = leftHeading;

        } else if (rightResult.hasTargets()) {
            // If only right camera has targets, use right camera values
            double rightSideDistance = PhotonUtils.calculateDistanceToTargetMeters(rightResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double rightNormalDistance = PhotonUtils.calculateDistanceToTargetMeters(rightResult, Constants.ReefConstants.REEF_TAG_HEIGHT);
            double rightHeading = PhotonUtils.calculateAngleToTargetDegrees(rightResult);

            // Use right camera results
            averageSideDistance = rightSideDistance;
            averageNormalDistance = rightNormalDistance;
            averageHeading = rightHeading;
        }

        // Output the averaged results (for testing purposes)
        System.out.println("Average Side Distance: " + averageSideDistance);
        System.out.println("Average Normal Distance: " + averageNormalDistance);
        System.out.println("Average Heading: " + averageHeading);
    }

    // Drive function (assuming this is a placeholder)
    public void autoDriveToReef() {
        // Use the average heading and distance to drive towards the Reef
        // Apply the P multiplier to scale the output based on the distance and heading
        double distanceError = averageNormalDistance; // Normal distance error
        double headingError = averageHeading; // Heading error

        double distanceAdjustment = distanceError * P;
        double headingAdjustment = headingError * P;

        // Call your robot's drive system here to apply the adjustments
        // For example, let's assume you have a drive method like:
        // swerveDrive.drive(distanceAdjustment, headingAdjustment);
        // Adjust for distance and heading errors
        swerveDrive.drive(distanceAdjustment, headingAdjustment);
    }

    // Getters for the calculated values (for other subsystems or usage)
    public double getAverageSideDistance() {
        return averageSideDistance;
    }

    public double getAverageNormalDistance() {
        return averageNormalDistance;
    }

    public double getAverageHeading() {
        return averageHeading;
    }
}
