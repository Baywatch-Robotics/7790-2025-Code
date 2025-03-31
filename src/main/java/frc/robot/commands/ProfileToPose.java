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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ObstacleAvoidanceConstants;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeomUtil;

public class ProfileToPose extends Command {
    // Use constants from DriveToPoseConstants
    private static final double drivekP = DriveToPoseConstants.DRIVE_KP;
    private static final double drivekD = DriveToPoseConstants.DRIVE_KD;
    private static final double thetakP = DriveToPoseConstants.THETA_KP;
    private static final double thetakD = DriveToPoseConstants.THETA_KD;

    // Default values - will be updated dynamically based on distance
    private double driveMaxVelocity = DriveToPoseConstants.DRIVE_MAX_VELOCITY;
    private double driveMaxAcceleration = DriveToPoseConstants.DRIVE_MAX_ACCELERATION;
    private static final double thetaMaxVelocity = DriveToPoseConstants.THETA_MAX_VELOCITY;
    private static final double thetaMaxAcceleration = DriveToPoseConstants.THETA_MAX_ACCELERATION;
    
    // Tolerance values
    public static final double driveTolerance = DriveToPoseConstants.DRIVE_TOLERANCE;
    public static final double thetaTolerance = DriveToPoseConstants.THETA_TOLERANCE;
    
    private static final double ffMinRadius = 0.02;
    private static final double ffMaxRadius = 0.08;

    private SwerveSubsystem swerve;
    private Supplier<Pose2d> target;
    private ButtonBox buttonBox; // Added ButtonBox reference
    private boolean useButtonBox = false; // Flag to determine which source to use
    private boolean isFinalDestination = true; // Flag to indicate if this is the final destination
    
    // Obstacle avoidance flags and command reference
    private boolean useObstacleAvoidance = true; // Enable by default
    private ObstacleAvoidanceCommand obstacleAvoidanceCommand = null;
    private boolean usingObstacleAvoidance = false;
    
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
    
    // Original constructor for backward compatibility
    public ProfileToPose(SwerveSubsystem swerve, Supplier<Pose2d> target) {
        this(swerve, target, true); // Default to treating as final destination
    }
    
    // New constructor with isFinalDestination flag
    public ProfileToPose(SwerveSubsystem swerve, Supplier<Pose2d> target, boolean isFinalDestination) {
        this.swerve = swerve;
        this.target = target;
        this.useButtonBox = false;
        this.isFinalDestination = isFinalDestination;
        
        // Initialize controllers with default constraints
        initializeControllers();

        SmartDashboard.putString("cons", target.get().toString()); // Diagnostic
        SmartDashboard.putBoolean("Is Final Destination", isFinalDestination);
        addRequirements(swerve);
    }
    
    // ButtonBox constructor
    public ProfileToPose(SwerveSubsystem swerve, ButtonBox buttonBox) {
        this.swerve = swerve;
        this.buttonBox = buttonBox;
        this.useButtonBox = true;
        this.isFinalDestination = true; // ButtonBox targets are always treated as final destinations
        this.target = this::getTargetPose; // Use getTargetPose method as the supplier
        
        // Initialize controllers with default constraints
        initializeControllers();
        
        addRequirements(swerve);
    }
    
    // Method to disable obstacle avoidance if needed
    public ProfileToPose withoutObstacleAvoidance() {
        this.useObstacleAvoidance = false;
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
    
    /**
     * Determines if obstacle avoidance is needed to reach the target
     */
    private boolean needsObstacleAvoidance(Pose2d start, Pose2d target) {
        // Don't use obstacle avoidance if it's disabled or if this is a waypoint
        if (!useObstacleAvoidance || !isFinalDestination) {
            return false;
        }
        
        // Check if direct path intersects with reef
        Translation2d startPos = start.getTranslation();
        Translation2d endPos = target.getTranslation();
        
        double pathLength = startPos.getDistance(endPos);
        int numChecks = (int) Math.ceil(pathLength / ObstacleAvoidanceConstants.PATH_RESOLUTION);
        
        for (int i = 0; i <= numChecks; i++) {
            double t = (double) i / numChecks;
            Translation2d point = startPos.interpolate(endPos, t);
            if (isInReef(point)) {
                SmartDashboard.putString("Path Status", "Obstacle detected - using avoidance");
                return true; // Path intersects with reef, need obstacle avoidance
            }
        }
        
        SmartDashboard.putString("Path Status", "Clear path");
        return false; // Direct path is clear
    }
    
    /**
     * Checks if a point is inside the reef zone
     */
    private boolean isInReef(Translation2d point) {
        return point.getDistance(
            new Translation2d(ObstacleAvoidanceConstants.REEF_CENTER_X, ObstacleAvoidanceConstants.REEF_CENTER_Y)
        ) < ObstacleAvoidanceConstants.REEF_RADIUS;
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
        
        // Check if we need obstacle avoidance
        if (needsObstacleAvoidance(currentPose, targetPose)) {
            // Create and initialize obstacle avoidance command
            obstacleAvoidanceCommand = new ObstacleAvoidanceCommand(swerve, () -> targetPose);
            obstacleAvoidanceCommand.initialize();
            usingObstacleAvoidance = true;
            SmartDashboard.putBoolean("Using Obstacle Avoidance", true);
        } else {
            // Direct path navigation
            usingObstacleAvoidance = false;
            SmartDashboard.putBoolean("Using Obstacle Avoidance", false);
            
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
    }

    /**
     * Updates the speed constraints based on distance to target
     */
    private void updateSpeedConstraints(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        
        // Select appropriate constraints based on distance
        double newVelocity;
        double newAcceleration;
        
        // If this is not the final destination, always use the fastest speed
        if (!isFinalDestination) {
            newVelocity = DriveToPoseConstants.APPROACHING_MAX_VEL;
            newAcceleration = DriveToPoseConstants.APPROACHING_MAX_ACCEL;
            SmartDashboard.putString("Drive Speed Mode", "WAYPOINT (FULL SPEED)");
        }
        // For final destination, use dynamic speed based on distance
        else if (distance > DriveToPoseConstants.APPROACHING_DISTANCE_THRESHOLD) {
            // Far from target - use fastest speed
            newVelocity = DriveToPoseConstants.APPROACHING_MAX_VEL;
            newAcceleration = DriveToPoseConstants.APPROACHING_MAX_ACCEL;
            SmartDashboard.putString("Drive Speed Mode", "APPROACHING");
        } 
        else if (distance > DriveToPoseConstants.CLOSE_DISTANCE_THRESHOLD) {
            // Getting closer - use medium speed
            newVelocity = DriveToPoseConstants.CLOSE_MAX_VEL;
            newAcceleration = DriveToPoseConstants.CLOSE_MAX_ACCEL;
            SmartDashboard.putString("Drive Speed Mode", "CLOSE");
        } 
        else {
            // Very close - use slowest speed for precision
            newVelocity = DriveToPoseConstants.VERY_CLOSE_MAX_VEL;
            newAcceleration = DriveToPoseConstants.VERY_CLOSE_MAX_ACCEL;
            SmartDashboard.putString("Drive Speed Mode", "VERY CLOSE");
        }
        
        // Only update if the constraints have changed
        if (newVelocity != driveMaxVelocity || newAcceleration != driveMaxAcceleration) {
            driveMaxVelocity = newVelocity;
            driveMaxAcceleration = newAcceleration;
            
            // Update the constraints on the controller
            driveController.setConstraints(
                new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
                
            // Log the new values
            SmartDashboard.putNumber("Drive Max Velocity", driveMaxVelocity);
            SmartDashboard.putNumber("Drive Max Acceleration", driveMaxAcceleration);
        }
    }

    @Override
    public void execute() {
        // Skip execution completely if running is false (target was null)
        if (!running && !usingObstacleAvoidance) {
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
        
        // If using obstacle avoidance, delegate execution to it
        if (usingObstacleAvoidance && obstacleAvoidanceCommand != null) {
            obstacleAvoidanceCommand.execute();
            return;
        }
        
        // Direct path navigation code (existing execute logic)
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();
        
        // Update constraints based on current distance
        updateSpeedConstraints(currentPose, targetPose);

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        
        // THE MISSING PART - THIS IS WHERE IT WAS CUTTING OFF:
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
                
        // Log current distance for debugging
        SmartDashboard.putNumber("Distance to Target", currentDistance);
    }

    @Override
    public void end(boolean interrupted) {
        // If using obstacle avoidance, end that as well
        if (usingObstacleAvoidance && obstacleAvoidanceCommand != null) {
            obstacleAvoidanceCommand.end(interrupted);
        }
        running = false;
        
        
        // Stop the robot when the command ends
        swerve.drive(new ChassisSpeeds(0, 0, 0));
         // Lock the swerve modules to prevent drifting when command ends
         swerve.lock();
    }

    @Override
    public boolean isFinished() {
        // If using obstacle avoidance, delegate to its isFinished method
        if (usingObstacleAvoidance && obstacleAvoidanceCommand != null) {
            return obstacleAvoidanceCommand.isFinished();
        }
        
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