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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeomUtil;

public class FastProfileToPose extends Command {
    // Use constants from DriveToPoseConstants
    private static final double drivekP = DriveToPoseConstants.DRIVE_KP;
    private static final double drivekD = DriveToPoseConstants.DRIVE_KD;
    private static final double thetakP = DriveToPoseConstants.THETA_KP;
    private static final double thetakD = DriveToPoseConstants.THETA_KD;

    // Default values - will be updated dynamically based on distance
    private double driveMaxVelocity = 6.0;
    private double driveMaxAcceleration =  6.0;
    private static final double thetaMaxVelocity = DriveToPoseConstants.THETA_MAX_VELOCITY;
    private static final double thetaMaxAcceleration = DriveToPoseConstants.THETA_MAX_ACCELERATION;
    
    // Tolerance values
    public static final double driveTolerance = 0.03;
    public static final double thetaTolerance = DriveToPoseConstants.THETA_TOLERANCE;
    
    private static final double ffMinRadius = 0.01;
    private static final double ffMaxRadius = 0.08;

    private SwerveSubsystem swerve;
    private Supplier<Pose2d> target;
    private ButtonBox buttonBox; // Added ButtonBox reference
    private boolean useButtonBox = false; // Flag to determine which source to use
    
    // Controllers are now instance variables instead of static to allow for dynamic constraint updates
    private ProfiledPIDController driveController;
    private ProfiledPIDController thetaController;
    
    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;
    private Supplier<Pose2d> robot = () -> swerve.getPose();

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;
    
    // Add timer and rotation delay functionality
    private final Timer commandTimer = new Timer();
    private boolean useRotationDelay = false;
    private double rotationDelaySeconds = DriveToPoseConstants.AUTON_ROTATION_DELAY;
    
    // Original constructor for backward compatibility
    public FastProfileToPose(SwerveSubsystem swerve, Supplier<Pose2d> target) {
        this.swerve = swerve;
        this.target = target;
        this.useButtonBox = false;
        
        // Initialize controllers with default constraints
        initializeControllers();

        SmartDashboard.putString("cons", target.get().toString()); // Diagnostic
        addRequirements(swerve);
    }
    
    // New constructor that accepts ButtonBox directly
    public FastProfileToPose(SwerveSubsystem swerve, ButtonBox buttonBox) {
        this.swerve = swerve;
        this.buttonBox = buttonBox;
        this.useButtonBox = true;
        this.target = this::getTargetPose; // Use getTargetPose method as the supplier
        
        // Initialize controllers with default constraints
        initializeControllers();
        
        addRequirements(swerve);
    }
    
    // Method to enable rotation delay with default delay time
    public FastProfileToPose withRotationDelay() {
        this.useRotationDelay = true;
        this.rotationDelaySeconds = DriveToPoseConstants.AUTON_ROTATION_DELAY;
        return this;
    }
    
    // Method to enable rotation delay with custom delay time
    public FastProfileToPose withRotationDelay(double delaySeconds) {
        this.useRotationDelay = true;
        this.rotationDelaySeconds = delaySeconds;
        return this;
    }
    
    // Helper method to initialize controllers
    private void initializeControllers() {
        // Initialize with default constraints
        driveController = new ProfiledPIDController(drivekP, 0, drivekD,
            new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
            
        thetaController = new ProfiledPIDController(thetakP, 0, thetakD,
            new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
            
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
        
        // Start the timer for rotation delay
        commandTimer.reset();
        commandTimer.start();
        
        // Log rotation delay status
        SmartDashboard.putBoolean("Using Rotation Delay", useRotationDelay);
        if (useRotationDelay) {
            SmartDashboard.putNumber("Rotation Delay (seconds)", rotationDelaySeconds);
        }
        
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
        
        // Calculate initial distance and set initial constraints
        updateSpeedConstraints(currentPose, targetPose);
        
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

    /**
     * Updates the speed constraints based on distance to target
     */
    private void updateSpeedConstraints(Pose2d currentPose, Pose2d targetPose) {
                
        // Select appropriate constraints based on distance
        double newVelocity;
        double newAcceleration;
        
        
            newVelocity = driveMaxVelocity;
            newAcceleration = driveMaxAcceleration;
            SmartDashboard.putString("Drive Speed Mode", "APPROACHING");
        
        
        // Only update if the constraints have changed
        if (newVelocity != driveMaxVelocity || newAcceleration != driveMaxAcceleration) {
            driveMaxVelocity = newVelocity;
            driveMaxAcceleration = newAcceleration;
            
            // Update the constraints on the controller
            driveController.setConstraints(
                new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
        }
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
        
        // Update constraints based on current distance
        updateSpeedConstraints(currentPose, targetPose);

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

        // Calculate theta speed with rotation delay logic
        double thetaVelocity;
        
        // Apply rotation delay if enabled and still within delay period
        if (useRotationDelay && commandTimer.get() < rotationDelaySeconds) {
            // During delay period, don't rotate (zero angular velocity)
            thetaVelocity = 0.0;
            
            // Log that we're in delay period
            SmartDashboard.putBoolean("Rotation Delayed", true);
            SmartDashboard.putNumber("Rotation Delay Remaining", rotationDelaySeconds - commandTimer.get());
        } else {
            // After delay period (or if delay not enabled), calculate normal rotation
            thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                    + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            thetaErrorAbs =
                Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
            if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
            
            // Clear delay indicators if we were using delay
            if (useRotationDelay) {
                SmartDashboard.putBoolean("Rotation Delayed", false);
                SmartDashboard.putNumber("Rotation Delay Remaining", 0.0);
            }
        }

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
                
        // Log current distance for debugging
        SmartDashboard.putNumber("Distance to Target", currentDistance);
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        swerve.lock();
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

        // New static method that creates, schedules and returns immediately
        public static Command startAndReturnCommand(SwerveSubsystem swerve, ButtonBox buttonBox) {
            return edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
                // Create the command and schedule it
                FastProfileToPose command = new FastProfileToPose(swerve, buttonBox);
                command.schedule();
            });
        }
    
    // New static method that creates a command with rotation delay
    public static Command startAndReturnCommandWithRotationDelay(SwerveSubsystem swerve, ButtonBox buttonBox) {
        return edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
            // Create the command with rotation delay and schedule it
            FastProfileToPose command = new FastProfileToPose(swerve, buttonBox);
            command.withRotationDelay();
            command.schedule();
        });
    }
    
    // Method with custom delay time
    public static Command startAndReturnCommandWithRotationDelay(SwerveSubsystem swerve, ButtonBox buttonBox, double delaySeconds) {
        return edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
            // Create the command with custom rotation delay and schedule it
            FastProfileToPose command = new FastProfileToPose(swerve, buttonBox);
            command.withRotationDelay(delaySeconds);
            command.schedule();
        });
    }
}