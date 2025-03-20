// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.AprilTagVisionConstants;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;
  private boolean isClose = false;

  private boolean pathCanceled = false;

  // Add these fields to track drift and recalibration
  private int driftDetectionCounter = 0;
  private static final int DRIFT_DETECTION_THRESHOLD = 5; // Number of consecutive cycles with large drift before recalibration
  private static final double SEVERE_DRIFT_THRESHOLD = 1.0; // Distance in meters considered severe drift
  private static final double POSITION_JUMP_LIMIT = 0.1; // Max position change in meters per cycle to prevent jumps
  private static final double ROTATION_JUMP_LIMIT = Math.toRadians(5); // Max rotation change in radians per cycle
  private Pose2d lastAppliedPose = null;
  private boolean forceRecalibrationMode = false;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
//    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfdrig Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

  @Override
  public void periodic()
  {

    if(!isClose){
      addVisionMeasurementInitial();
    }
    else {
      // Modified to handle drift detection and recalibration
      integrateVisionMeasurements();
    }
  SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.15, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }


  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }


  

  public Command pathfindThenFollowPath(String pathName) {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        /*
        PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
        */

        PathConstraints constraints = new PathConstraints(1.0, 1.0, Units.degreesToRadians(120), Units.degreesToRadians(120)); //For testing
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public void setCancel(boolean value) {
    this.pathCanceled = value;
  }
  
  public boolean getCancel() {
    return this.pathCanceled;
  }
  
  public Command driveToPose(ButtonBox buttonBox, Elevator elevator) {
    return new Command() {
        private Command pathCommand;
        private PathConstraints lastConstraints = null;
        private TargetClass lastTarget = null;
        
        @Override
        public void initialize() {
            // Reset cancel flag once at the very start
            setCancel(false);
        }
        
        @Override
        public void execute() {
            // Get current target from the supplier
            TargetClass target = buttonBox.currentTargetClassSupplier.get();
            if (target == null) {
                target = TargetClass.GetTargetByName("C100");
            }
            
            // Compute target pose from the target
            Pose2d targetPose = new Pose2d(
                new Translation2d(target.getX(), target.getY()),
                Rotation2d.fromRadians(target.getZ())
            );
            Pose2d finalTargetPose = TargetClass.toPose2d(targetPose);
            
            double baseVelocity, baseAcceleration;
            
          
            baseVelocity = DriveToPoseConstants.VERY_CLOSE_MAX_VEL;
            baseAcceleration = DriveToPoseConstants.VERY_CLOSE_MAX_ACCEL;
            
            
            // Create the PathConstraints with the computed values
            PathConstraints currentConstraints = new PathConstraints(
              baseVelocity, 
              baseAcceleration,
                DriveToPoseConstants.MAX_ANGULAR_VEL,
                DriveToPoseConstants.MAX_ANGULAR_ACCEL
            );
            
            // Only update when either constraints or target have changed
            if (!currentConstraints.equals(lastConstraints) || !target.equals(lastTarget)) {
                if (pathCommand != null && !pathCommand.isFinished()) {
                    pathCommand.cancel();
                }
                
                // Create and schedule the new pathfinding command using the computed constraints
                pathCommand = AutoBuilder.pathfindToPose(finalTargetPose, currentConstraints,
                    edu.wpi.first.units.Units.MetersPerSecond.of(0));
                pathCommand.schedule();
                
                // Cache new values
                lastConstraints = currentConstraints;
                lastTarget = target;
            }
            
            // Check cancellation flag during this cycle
            if (getCancel() && pathCommand != null && !pathCommand.isFinished()) {
                pathCommand.cancel();
            }
        }
        
        @Override
        public boolean isFinished() {
            return (pathCommand != null && pathCommand.isFinished()) || getCancel();
        }
        
        @Override
        public void end(boolean interrupted) {
            if (pathCommand != null) {
                pathCommand.cancel();
            }
            setCancel(false);
        }
    };
}


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }
*/
  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  // Replace the existing addVisionMeasurement with this improved vision integration method
  private void integrateVisionMeasurements() {
    if (DriverStation.isDisabled()) {
      // Reset drift detection in disabled mode since we're using more aggressive corrections
      driftDetectionCounter = 0;
      forceRecalibrationMode = false;
      
      // Original disabled mode logic (simplified)
      Pose2d robotPose = swerveDrive.getPose();
      Optional<Pose3d> estimatedPose3d = AprilTagVision.getBestPoseEstimate(robotPose);
      
      if (estimatedPose3d.isPresent()) {
        Pose2d newPose = estimatedPose3d.get().toPose2d();
        swerveDrive.addVisionMeasurement(newPose, Timer.getFPGATimestamp());
      }
      return;
    }
    
    // ENABLED mode - more careful integration
    Pose2d robotPose = swerveDrive.getPose();
    Optional<Pose3d> estimatedPose3d = AprilTagVision.getBestPoseEstimate(robotPose);
    
    if (!estimatedPose3d.isPresent()) {
      // No vision - increment drift counter if we were in recalibration mode
      if (forceRecalibrationMode) {
        driftDetectionCounter++;
        // Exit recalibration mode if we have no vision for too long
        if (driftDetectionCounter > DRIFT_DETECTION_THRESHOLD * 2) {
          forceRecalibrationMode = false;
          driftDetectionCounter = 0;
        }
      }
      return;
    }
    
    Pose2d visionPose = estimatedPose3d.get().toPose2d();
    double timestamp = Timer.getFPGATimestamp();
    double distance = visionPose.getTranslation().getDistance(robotPose.getTranslation());
    
    // Log metrics
    SmartDashboard.putNumber("Vision/Pose Error", distance);
    SmartDashboard.putBoolean("Vision/Recalibration Mode", forceRecalibrationMode);
    SmartDashboard.putNumber("Vision/Drift Counter", driftDetectionCounter);
    
    // Check for severe drift - if detected, increment counter
    if (distance > SEVERE_DRIFT_THRESHOLD) {
      driftDetectionCounter++;
      SmartDashboard.putString("Vision/Status", "Potential Drift Detected");
      
      if (driftDetectionCounter >= DRIFT_DETECTION_THRESHOLD) {
        // Enter recalibration mode after consecutive drift detections
        forceRecalibrationMode = true;
      }
    } else {
      // Reset drift counter when measurements are close
      driftDetectionCounter = 0;
      if (forceRecalibrationMode) {
        // Exit recalibration mode once we're close again
        forceRecalibrationMode = false;
      }
    }
    
    // Different integration strategy based on mode
    if (forceRecalibrationMode) {
      // In recalibration mode - do a partial reset toward vision measurement
      // This helps avoid sudden jumps while still moving toward correct position
      double blendFactor = 0.3; // Adjust toward vision by 30% each cycle
      Translation2d blendedTranslation = robotPose.getTranslation().interpolate(
          visionPose.getTranslation(), blendFactor);
      
      // Blend rotation more carefully to avoid spinning
      double angleDiff = visionPose.getRotation().minus(robotPose.getRotation()).getRadians();
      // Normalize to [-π, π]
      while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
      while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
      
      // Limit rotation change per cycle
      double rotationChange = Math.signum(angleDiff) * Math.min(Math.abs(angleDiff) * blendFactor, ROTATION_JUMP_LIMIT);
      Rotation2d blendedRotation = robotPose.getRotation().plus(new Rotation2d(rotationChange));
      
      // Create blended pose and apply
      Pose2d blendedPose = new Pose2d(blendedTranslation, blendedRotation);
      SmartDashboard.putString("Vision/Status", "Recalibrating");
      
      // Apply the blended measurement
      swerveDrive.addVisionMeasurement(blendedPose, timestamp);
      lastAppliedPose = blendedPose;
    } else {
      // Normal integration mode with confidence-based standard deviations
      // This is similar to the original code but with added jump protection
      
      // Calculate confidence based on distance
      double confidence = calculateConfidence(distance);
      
      // Create the standard deviation matrix
      Matrix<N3, N1> stdDevs = createStdDevMatrix(confidence);
      
      // Apply the measurement if it's not a jump and has reasonable confidence
      if (confidence > 0.1 && !wouldCauseJump(robotPose, visionPose)) {
        swerveDrive.addVisionMeasurement(visionPose, timestamp, stdDevs);
        lastAppliedPose = visionPose;
        SmartDashboard.putString("Vision/Status", "Normal Integration");
      } else {
        SmartDashboard.putString("Vision/Status", "Measurement Rejected");
      }
    }
  }
  
  // Helper method to check if applying a vision measurement would cause a jump
  private boolean wouldCauseJump(Pose2d currentPose, Pose2d newPose) {
    if (lastAppliedPose == null) {
      return false; // First measurement, can't be a jump
    }
    
    // Check position jump
    double positionChange = newPose.getTranslation().getDistance(currentPose.getTranslation());
    if (positionChange > POSITION_JUMP_LIMIT) {
      return true;
    }
    
    // Check rotation jump
    double rotationChange = Math.abs(newPose.getRotation().minus(currentPose.getRotation()).getRadians());
    // Normalize to [0, π]
    while (rotationChange > Math.PI) rotationChange -= 2 * Math.PI;
    rotationChange = Math.abs(rotationChange);
    
    return rotationChange > ROTATION_JUMP_LIMIT;
  }
  
  // Helper method to calculate confidence based on distance between odometry and vision
  private double calculateConfidence(double distance) {
    if (distance <= AprilTagVisionConstants.MAX_VISION_DISTANCE_TRUSTED) {
      return 1.0;
    } else if (distance <= AprilTagVisionConstants.MAX_VISION_DISTANCE_CONSIDERED) {
      return 1.0 - ((distance - AprilTagVisionConstants.MAX_VISION_DISTANCE_TRUSTED) 
          / (AprilTagVisionConstants.MAX_VISION_DISTANCE_CONSIDERED - AprilTagVisionConstants.MAX_VISION_DISTANCE_TRUSTED));
    }
    return 0.0;
  }
  
  // Helper method to create standard deviation matrix based on confidence
  private Matrix<N3, N1> createStdDevMatrix(double confidence) {
    // Adjust minimum confidence to avoid division by zero
    confidence = Math.max(AprilTagVisionConstants.MIN_CONFIDENCE_VALUE, confidence);
    
    // Scale standard deviations inversely with confidence
    double xyStdDev = AprilTagVisionConstants.BASE_XY_STD_DEV / confidence;
    double rotStdDev = AprilTagVisionConstants.BASE_ROT_STD_DEV / confidence;
    
    // Create the standard deviation matrix
    Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, xyStdDev);    // X standard deviation
    stdDevs.set(1, 0, xyStdDev);    // Y standard deviation 
    stdDevs.set(2, 0, rotStdDev);   // Rotation standard deviation
    
    return stdDevs;
  }
  
  // Modify the existing addVisionMeasurementInitial method for more robustness
  public void addVisionMeasurementInitial() {
    Pose2d robotPose = swerveDrive.getPose();
    Optional<Pose3d> estimatedPose3d = AprilTagVision.getBestPoseEstimate(robotPose);
    
    if (estimatedPose3d.isPresent()) {
      Pose2d newPose = estimatedPose3d.get().toPose2d();
      double distance = newPose.getTranslation().getDistance(robotPose.getTranslation());
      
      // Check if we have a multi-tag detection or high confidence single tag
      boolean isMultiTag = false;
      boolean isLowAmbiguity = false;
      
      // Get the most recent camera results to check tag count
      var rightResult = AprilTagVision.getLastRightCamResult();
      var leftResult = AprilTagVision.getLastLeftCamResult();
      
      if (rightResult.isPresent() && rightResult.get().targetsUsed.size() > 1) {
        isMultiTag = true;
      } else if (leftResult.isPresent() && leftResult.get().targetsUsed.size() > 1) {
        isMultiTag = true;
      } else if (rightResult.isPresent() && rightResult.get().targetsUsed.size() == 1 && 
                 rightResult.get().targetsUsed.get(0).getPoseAmbiguity() < 0.05) {
        isLowAmbiguity = true;
      } else if (leftResult.isPresent() && leftResult.get().targetsUsed.size() == 1 && 
                 leftResult.get().targetsUsed.get(0).getPoseAmbiguity() < 0.05) {
        isLowAmbiguity = true;
      }
      
      // Only reset odometry if we have high confidence in the vision measurements
      if (distance >= 0.5 && (isMultiTag || isLowAmbiguity)) {
        SmartDashboard.putString("Vision/Init Status", "Resetting Odometry");
        swerveDrive.resetOdometry(newPose);
        lastAppliedPose = newPose;
      } else if (distance < 0.5) {
        isClose = true;
        SmartDashboard.putString("Vision/Init Status", "Switching to Normal Mode");
      } else {
        SmartDashboard.putString("Vision/Init Status", "Waiting for Confident Measurement");
      }
    }
  }

    /**
     * Visualize the target pose on the field
     * @param targetPose The pose to visualize
     */
    public void visualizeTargetPose(Pose2d targetPose) {
        swerveDrive.field.getObject("targetPose").setPose(targetPose);
    }
    
    /**
     * Clear the target visualization from the field
     * This removes the target marker without affecting the queue
     */
    public void clearTargetVisualization() {
        // Set an invisible pose far away to effectively hide it
        // Using a position outside the field boundary keeps it hidden
        swerveDrive.field.getObject("targetPose").setPose(
            new Pose2d(-100, -100, new Rotation2d(0))
        );
    }

    /**
     * Starts the drive to pose command and returns immediately.
     * The drive command will continue running in the background.
     * You can use your triggers (veryCloseTrigger, etc.) to determine when the robot is at the target.
     * 
     * @param buttonBox The button box containing target information
     * @return A command that starts the drive process and completes immediately
     */
    public Command startDriveToPose(ButtonBox buttonBox, Elevator elevator) {
        return Commands.runOnce(() -> {
            driveToPose(buttonBox, elevator).schedule();
        });
    }
}