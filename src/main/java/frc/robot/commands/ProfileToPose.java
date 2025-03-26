package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeomUtil;

public class ProfileToPose extends Command {
    private static final double drivekP = 0.9;
    private static final double drivekD = 0.01;
    private static final double thetakP = 4.0;
    private static final double thetakD = 0.0;

    private static final double driveMaxVelocity = 0.7;
    private static final double driveMaxAcceleration = 1;
    private static final double thetaMaxVelocity = Units.degreesToRadians(720.0);
    private static final double thetaMaxAcceleration = 10.0;
    public static final double driveTolerance = 0.01;
    public static final double thetaTolerance = Units.degreesToRadians(1.0);
    
    private static final double ffMinRadius = 0.02;
    private static final double ffMaxRadius = 0.08;

    private SwerveSubsystem swerve;
    private Supplier<Pose2d> target;
    private ButtonBox buttonBox; // Added ButtonBox reference
    private boolean useButtonBox = false; // Flag to determine which source to use
    
    private static final ProfiledPIDController driveController = new ProfiledPIDController(drivekP, 0, drivekD,
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    private static final ProfiledPIDController thetaController = new ProfiledPIDController(thetakP, 0, thetakD,
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;
    private Supplier<Pose2d> robot = () -> swerve.getPose();

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;
    
    // Original constructor for backward compatibility
    public ProfileToPose(SwerveSubsystem swerve, Supplier<Pose2d> target) {
        this.swerve = swerve;
        this.target = target;
        this.useButtonBox = false;

        SmartDashboard.putString("cons", target.get().toString()); // Diagnostic
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }
    
    // New constructor that accepts ButtonBox directly
    public ProfileToPose(SwerveSubsystem swerve, ButtonBox buttonBox) {
        this.swerve = swerve;
        this.buttonBox = buttonBox;
        this.useButtonBox = true;
        this.target = this::getTargetPose; // Use getTargetPose method as the supplier

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }
    
    // Helper method to get the target pose from the ButtonBox
    private Pose2d getTargetPose() {
        if (buttonBox == null) {
            return new Pose2d(); // Return origin if buttonBox is null
        }
        
        TargetClass target = buttonBox.peekNextTarget();
        if (target != null) {
            Pose2d targetPose = new Pose2d(
                new Translation2d(target.getX(), target.getY()),
                Rotation2d.fromRadians(target.getZ())
            );
            
            SmartDashboard.putString("Drive Target", target.getName());
            return TargetClass.toPose2d(targetPose);
        } else {
            SmartDashboard.putString("Drive Target", "NULL");
            return swerve.getPose(); // Return current pose to effectively stop movement
        }
    }

    @Override
    public void initialize() {
        Pose2d targetPose = target.get();
        SmartDashboard.putString("init", targetPose.toString()); // Diagnostic
        
        // Check if there's no valid target (when using ButtonBox and target is null)
        if (useButtonBox && buttonBox.peekNextTarget() == null) {
            SmartDashboard.putBoolean("Null Target Detected", true);
            // Mark the command as complete immediately
            running = false;
            return;
        }
        SmartDashboard.putBoolean("Null Target Detected", false);
        
        // Normal initialization continues if target is valid
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = swerve.getRobotVelocity();
        Translation2d linearFieldVelocity =
            new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
            currentPose.getTranslation().getDistance(targetPose.getTranslation()),
            Math.min(
                0.0,
                -linearFieldVelocity
                    .rotateBy(
                        targetPose
                            .getTranslation()
                            .minus(currentPose.getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));
        thetaController.reset(
            currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
        running = true;
    }

    @Override
    public void execute() {
        // Skip execution completely if running is false (target was null)
        if (!running) {
            return;
        }
        
        // Check if drive should be canceled due to joystick input
        if (swerve.getCancel()) {
            running = false;
            return;
        }
        
        // Check if using ButtonBox and target is now null (target changed during execution)
        if (useButtonBox && buttonBox.peekNextTarget() == null) {
            running = false;
            return;
        }
        
        // Regular execution continues
        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()),
            driveController.getSetpoint().velocity);
        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation =
            new Pose2d(
                    targetPose.getTranslation(),
                    new Rotation2d(
                        Math.atan2(
                            currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                            currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs =
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        Translation2d driveVelocity =
            new Pose2d(
                    Translation2d.kZero,
                    new Rotation2d(
                        Math.atan2(
                            currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                            currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
        driveVelocity =
            driveVelocity.interpolate(linearFF.get().times(4.69), linearS);
        thetaVelocity =
            MathUtil.interpolate(
                thetaVelocity, omegaFF.getAsDouble() * 4.69/Math.hypot(10.75, 10.75), thetaS);

        // Command speeds (ROBOT RELATIVE)
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
    }

    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
            && Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }

    @Override
    public boolean isFinished() {
        // If no valid target (when using ButtonBox and target is null)
        if (useButtonBox && buttonBox.peekNextTarget() == null) {
            return true;
        }
        
        // Check if canceled by joystick movement
        if (swerve.getCancel()) {
            return true;
        }
        
        // Otherwise use normal completion criteria
        return !running || (running && driveController.atGoal() && thetaController.atGoal());
    }
}