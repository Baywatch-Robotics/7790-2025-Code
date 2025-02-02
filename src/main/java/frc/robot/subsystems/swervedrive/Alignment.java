package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlignmentConstants;

public class Alignment extends SubsystemBase {

    public static class JoystickCommands {
        public final double driveX;
        public final double driveY;
        public final double headingX;
        public final double headingY;

        public JoystickCommands(double driveX, double driveY, double headingX, double headingY) {
            this.driveX = driveX;
            this.driveY = driveY;
            this.headingX = headingX;
            this.headingY = headingY;
        }
    }

    public static JoystickCommands driveToPose(Pose2d currentPose, Pose2d targetPose) {
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();

        double errorXField = targetTranslation.getX() - currentTranslation.getX();
        double errorYField = targetTranslation.getY() - currentTranslation.getY();

        double robotHeading = currentPose.getRotation().getRadians();
        double cosHeading = Math.cos(-robotHeading);
        double sinHeading = Math.sin(-robotHeading);

        double errorXRobot = errorXField * cosHeading - errorYField * sinHeading;
        double errorYRobot = errorXField * sinHeading + errorYField * cosHeading;

        double driveX = MathUtil.clamp(AlignmentConstants.P * errorXRobot, -1.0, 1.0);
        double driveY = MathUtil.clamp(AlignmentConstants.P * errorYRobot, -1.0, 1.0);

        double targetHeading = targetPose.getRotation().getRadians();
        double headingX = Math.sin(targetHeading);
        double headingY = Math.cos(targetHeading);

        return new JoystickCommands(driveX, driveY, headingX, headingY);
    }
}