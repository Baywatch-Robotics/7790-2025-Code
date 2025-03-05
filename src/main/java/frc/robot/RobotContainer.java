// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeShooterConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Algae.AlgaeShooter;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.Coral.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveInputStream;
import frc.robot.Constants.SpeedConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final CommandJoystick buttonBox1 = new CommandJoystick(1);
  private final CommandJoystick buttonBox2 = new CommandJoystick(2);
  final CommandXboxController opXbox = new CommandXboxController(3);

  // Add variables for smooth speed transition
  private float targetDriveSpeed = 0;
  private float actualDriveSpeed = 0;

  private boolean isClose = false;
  private boolean isVeryClose = false;
  private boolean isApproaching = false;
  private boolean isLinedUp = false;

  // Add zone status tracking
  private boolean isInReefZone = false;
  private boolean isInCoralStationLeftZone = false;  // Consistent naming
  private boolean isInCoralStationRightZone = false; // Consistent naming

  
  DoubleSupplier headingXAng = () -> -driverXbox.getRightX() * .8;
  DoubleSupplier angSpeed;

  DoubleSupplier driveX;
  DoubleSupplier driveY;
  DoubleSupplier headingX = () -> -driverXbox.getRightX();
  DoubleSupplier headingY = () -> -driverXbox.getRightY();

  DoubleSupplier elevatorUpDown = () -> opXbox.getRightY();
  //DoubleSupplier algaeArmTriggerUp = () -> opXbox.getLeftTriggerAxis();
  //DoubleSupplier algaeArmTriggerDown = () -> opXbox.getLeftTriggerAxis();
  DoubleSupplier shooterArmUpDown = () -> opXbox.getLeftY();
  DoubleSupplier shooterPivotUpDown = () -> opXbox.getLeftX(); //Questionable Name Practices... Shooter Pivot UP DOWN not Left Right??

  DoubleSupplier climberUpDown = () -> opXbox.getRightX();
  
  // Add suppliers for algae shooter triggers
  DoubleSupplier algaeShooterIntake = () -> driverXbox.getLeftTriggerAxis();
  DoubleSupplier algaeShooterOutake = () -> driverXbox.getRightTriggerAxis();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  
  
  private final AlgaeArm algaeArm = new AlgaeArm();
  private final AlgaeShooter algaeShooter = new AlgaeShooter();
  //private final Scope scope = new Scope();
  private final Shooter shooter = new Shooter();
  private final ShooterArm shooterArm = new ShooterArm();
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator(this);

  //private final Funnel funnel = new Funnel();
  //private final LED LED = new LED();
  private final ButtonBox buttonBox = new ButtonBox();

  // Triggers for proximity detection
  public Trigger approachingTrigger = new Trigger(() -> isApproaching);
  public Trigger closeTrigger = new Trigger(() -> isClose);
  public Trigger veryCloseTrigger = new Trigger(() -> isVeryClose);
  public Trigger linedUpTrigger = new Trigger(() -> isLinedUp);
  
  // Triggers for zone detection
  public Trigger reefZoneTrigger = new Trigger(() -> isInReefZone);
  public Trigger coralStationLeftTrigger = new Trigger(() -> isInCoralStationLeftZone);
  public Trigger coralStationRightTrigger = new Trigger(() -> isInCoralStationRightZone);
  public Trigger anyZoneTrigger = new Trigger(() -> isInReefZone || isInCoralStationLeftZone || isInCoralStationRightZone);

  // Track positioning accuracy
  private boolean isAtTargetPosition = false;
  private boolean isAtTargetRotation = false;
  private boolean isAtTarget = false;
  private double positionTolerance = Constants.DriveToPoseConstants.POSITION_TOLERANCE; 
  private double rotationTolerance = Constants.DriveToPoseConstants.ROTATION_TOLERANCE;
  
  // Trigger for when the robot is at the target position
  public Trigger atTargetPositionTrigger = new Trigger(() -> isAtTargetPosition);
  public Trigger atTargetRotationTrigger = new Trigger(() -> isAtTargetRotation);
  public Trigger atTargetTrigger = new Trigger(() -> isAtTarget);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driveY.getAsDouble(),
  () -> driveX.getAsDouble())
.withControllerRotationAxis(() -> angSpeed.getAsDouble())
.deadband(Constants.DEADBAND)
.scaleTranslation(1)
.allianceRelativeControl(true);

/**
* Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
*/
SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(headingX,
                               headingY).headingWhile(true);

/**
* Clone's the angular velocity input stream and converts it to a robotRelative input stream.
*/
SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
.allianceRelativeControl(false);

SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(Constants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                 .withControllerHeadingAxis(() ->
                                                Math.sin(
                                                    driverXbox.getRawAxis(
                                                        2) *
                                                    Math.PI) *
                                                (Math.PI *
                                                 2),
                                            () ->
                                                Math.cos(
                                                    driverXbox.getRawAxis(
                                                        2) *
                                                    Math.PI) *
                                                (Math.PI *
                                                 2))
                 .headingWhile(true);

/*
// Create an input stream using values provided by the ButtonBox.
SwerveInputStream driveButtonBoxInput =
    SwerveInputStream.of(drivebase.getSwerveDrive(),
        // For x and y inputs, get values from ButtonBox suppliers.
        () -> buttonBox.getFieldOrientedSuppliers().x.getAsDouble(),
        () -> buttonBox.getFieldOrientedSuppliers().y.getAsDouble())
    // For rotation, use the rotation suppliers from ButtonBox.
    .withControllerHeadingAxis(
        () -> buttonBox.getFieldOrientedSuppliers().rotationX.getAsDouble(),
        () -> buttonBox.getFieldOrientedSuppliers().rotationY.getAsDouble())
    .deadband(Constants.DEADBAND)
    .scaleTranslation(1.0)
    .allianceRelativeControl(true)
    // Optionally, if you want to drive with a heading-control mode:
    .headingWhile(true);
  */
  /*  SwerveInputStream driveAngularVelocityDriveToPose = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driverXbox.getLeftY() * -1,
  () -> driverXbox.getLeftX() * -1)
.withControllerRotationAxis(headingXAng)
.deadband(Constants.DEADBAND)
.scaleTranslation(driveSpeed)
.allianceRelativeControl(true)
.driveToPose(TargetClass.toPose2dSupplier(buttonBox), DriveToPoseConstants.xProfiledPID, DriveToPoseConstants.yProfiledPID)
.driveToPoseEnabled(driveToPoseEnabled);
  */

  // Track distance to target for PID adjustment
  private double distanceToTarget = Double.POSITIVE_INFINITY;
  private double angleToTarget = Double.POSITIVE_INFINITY;
  
  // Keep track of when controllers need to be updated
  private boolean pidControllersNeedUpdate = true;
  private ProfiledPIDController currentXController;
  private ProfiledPIDController currentRotController;
  
  // Dynamic PID controller suppliers
  private Supplier<ProfiledPIDController> driveToPoseXControllerSupplier = () -> {
      // Calculate distance to target for constraint selection
      TargetClass target = buttonBox.peekNextTarget();
      
      // Update distance to target for constraint calculation
      Pose2d robotPose = drivebase.getPose();
      distanceToTarget = Double.POSITIVE_INFINITY;
      
      if (target != null) {
          Pose2d targetPose = new Pose2d(target.getX(), target.getY(), new Rotation2d(target.getZ()));
          Pose2d allianceAdjustedPose = TargetClass.toPose2d(targetPose);
          distanceToTarget = robotPose.getTranslation().getDistance(allianceAdjustedPose.getTranslation());
          
          SmartDashboard.putString("Target Name", target.getName());
          SmartDashboard.putBoolean("Target Available", true);
      } else {
          SmartDashboard.putString("Target Name", "None");
          SmartDashboard.putBoolean("Target Available", false);
      }

      // Get current constraints based on distance
      Constraints currentConstraints = getTranslationConstraintsForDistance(distanceToTarget);
      
      // If controller doesn't exist, create it
      if (currentXController == null || pidControllersNeedUpdate) {
          currentXController = new ProfiledPIDController(
              Constants.DriveToPoseConstants.TRANSLATION_P,
              Constants.DriveToPoseConstants.TRANSLATION_I,
              Constants.DriveToPoseConstants.TRANSLATION_D,
              currentConstraints
          );
          currentXController.setTolerance(positionTolerance);
      } else {
          // Otherwise, just update the constraints
          currentXController.setConstraints(currentConstraints);
      }
      
      // Always display PID and constraint values
      SmartDashboard.putNumber("X Controller P", Constants.DriveToPoseConstants.TRANSLATION_P);
      SmartDashboard.putNumber("X Max Vel", currentConstraints.maxVelocity);
      SmartDashboard.putNumber("X Max Accel", currentConstraints.maxAcceleration);
      
      return currentXController;
  };
  
  private Supplier<ProfiledPIDController> driveToPoseRotControllerSupplier = () -> {
      // Calculate distance to target for constraint selection
      TargetClass target = buttonBox.peekNextTarget();
      
      // Update distance to target for constraint calculation
      Pose2d robotPose = drivebase.getPose();
      distanceToTarget = Double.POSITIVE_INFINITY;
      angleToTarget = 0;
      
      if (target != null) {
          Pose2d targetPose = new Pose2d(target.getX(), target.getY(), new Rotation2d(target.getZ()));
          Pose2d allianceAdjustedPose = TargetClass.toPose2d(targetPose);
          distanceToTarget = robotPose.getTranslation().getDistance(allianceAdjustedPose.getTranslation());
          
          // Calculate angle to target
          angleToTarget = Math.atan2(
              allianceAdjustedPose.getY() - robotPose.getY(),
              allianceAdjustedPose.getX() - robotPose.getX()
          ) - robotPose.getRotation().getRadians();
          // Normalize angle
          angleToTarget = Math.atan2(Math.sin(angleToTarget), Math.cos(angleToTarget));
      }

      // Get current constraints based on distance
      Constraints currentConstraints = getRotationConstraintsForDistance(distanceToTarget);
      
      // If controller doesn't exist, create it
      if (currentRotController == null || pidControllersNeedUpdate) {
          currentRotController = new ProfiledPIDController(
              Constants.DriveToPoseConstants.ROTATION_P,
              Constants.DriveToPoseConstants.ROTATION_I,
              Constants.DriveToPoseConstants.ROTATION_D,
              currentConstraints
          );
          currentRotController.setTolerance(rotationTolerance);
          currentRotController.enableContinuousInput(-Math.PI, Math.PI);
      } else {
          // Otherwise, just update the constraints
          currentRotController.setConstraints(currentConstraints);
      }
      
      // Always display PID and constraint values
      SmartDashboard.putNumber("Rot Controller P", Constants.DriveToPoseConstants.ROTATION_P);
      SmartDashboard.putNumber("Rot Max Vel", currentConstraints.maxVelocity);
      SmartDashboard.putNumber("Rot Max Accel", currentConstraints.maxAcceleration);
      
      return currentRotController;
  };

  // Create a stream that includes drive-to-pose capability with dynamic controllers
  SwerveInputStream driveToPoseStream = driveDirectAngle.copy().driveToPose(
      () -> {
          // Flag that controllers should be updated
          pidControllersNeedUpdate = true;
          
          // Get the target pose and update distance metrics
          TargetClass target = buttonBox.peekNextTarget();
          
          if (target != null) {
              Pose2d targetPose = new Pose2d(target.getX(), target.getY(), new Rotation2d(target.getZ()));
              Pose2d allianceAdjustedPose = TargetClass.toPose2d(targetPose);
              
              // Visualize the target pose
              drivebase.visualizeTargetPose(allianceAdjustedPose);
              
              // Get current robot pose
              Pose2d robotPose = drivebase.getPose();
              
              // Update distance and angle metrics
              distanceToTarget = robotPose.getTranslation().getDistance(allianceAdjustedPose.getTranslation());
              angleToTarget = Math.atan2(
                  allianceAdjustedPose.getY() - robotPose.getY(),
                  allianceAdjustedPose.getX() - robotPose.getX()
              ) - robotPose.getRotation().getRadians();
              // Normalize angle
              angleToTarget = Math.atan2(Math.sin(angleToTarget), Math.cos(angleToTarget));
              
              SmartDashboard.putNumber("Distance To Target", distanceToTarget);
              SmartDashboard.putNumber("Angle To Target (deg)", Units.radiansToDegrees(angleToTarget));
              SmartDashboard.putString("Target Name", target.getName());
              SmartDashboard.putBoolean("Target Available", true);
              
              return allianceAdjustedPose;
          } else {
              SmartDashboard.putString("Target Name", "None");
              SmartDashboard.putBoolean("Target Available", false);
              
              // Reset distance values when no target
              distanceToTarget = Double.POSITIVE_INFINITY;
              angleToTarget = 0;
              
              // Force controllers to update with zero values next cycle
              pidControllersNeedUpdate = true;
          }
          // Default pose if no target - this should not be used since PIDs are zero
          return new Pose2d();
      },
      driveToPoseXControllerSupplier.get(),
      driveToPoseRotControllerSupplier.get()
  );

  // Create drive commands
  public Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  public Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  public Command driveFieldOrientedDriveToPose = drivebase.driveFieldOriented(driveToPoseStream);

  // Track whether drive-to-pose is currently enabled
  private boolean isDriveToPoseEnabled = false;

  // Command to enable drive-to-pose
  public Command enableDriveToPoseCommand = Commands.runOnce(() -> {
    driveToPoseStream.driveToPoseEnabled(true);
    isDriveToPoseEnabled = true;
    SmartDashboard.putBoolean("Drive To Pose Enabled", true);
  });

  // Command to disable drive-to-pose
  public Command disableDriveToPoseCommand = Commands.runOnce(() -> {
    driveToPoseStream.driveToPoseEnabled(false);
    isDriveToPoseEnabled = false;
    SmartDashboard.putBoolean("Drive To Pose Enabled", false);
  });

  // Create a command to temporarily run drive-to-pose without canceling default command
  private Command tempDriveToPoseCommand = Commands.sequence(
    // Enable drive-to-pose mode and reset controllers
    Commands.runOnce(() -> {
        pidControllersNeedUpdate = true; // Force controller update
        currentXController = null; // Reset controllers to ensure they're recreated
        currentRotController = null;
        driveToPoseStream.driveToPoseEnabled(true);
        isDriveToPoseEnabled = true;
        
        // Reset target position status
        isAtTargetPosition = false;
        isAtTargetRotation = false;
        isAtTarget = false;
        
        SmartDashboard.putBoolean("Drive To Pose Active", true);
        SmartDashboard.putBoolean("At Target Position", false);
        SmartDashboard.putBoolean("At Target Rotation", false);
        SmartDashboard.putBoolean("At Target", false);
        
        // Check if we have a target immediately
        if (buttonBox.peekNextTarget() == null) {
            SmartDashboard.putString("Drive Status", "No Target Available");
        } else {
            SmartDashboard.putString("Drive Status", "Target Found - Driving");
        }
    }),
    // Run drive-to-pose until interrupted
    driveFieldOrientedDriveToPose,
    // Disable drive-to-pose when done (this runs even if interrupted)
    Commands.runOnce(() -> {
        driveToPoseStream.driveToPoseEnabled(false);
        isDriveToPoseEnabled = false;
        SmartDashboard.putBoolean("Drive To Pose Active", false);
        SmartDashboard.putString("Drive Status", "Normal Driving");
        
        // Reset target position status
        isAtTargetPosition = false;
        isAtTargetRotation = false;
        isAtTarget = false;
        SmartDashboard.putBoolean("At Target Position", false);
        SmartDashboard.putBoolean("At Target Rotation", false);
        SmartDashboard.putBoolean("At Target", false);
        
        // Clear the target visualization when command ends
        drivebase.clearTargetPose();
    })
).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);

  public Command leftAuto = CommandFactory.LeftAutonCommand(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this);
    public Command rightAuto = CommandFactory.RightAutonCommand(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this);

    SendableChooser<Command> chooser = new SendableChooser<>();
/**
* The container for the robot. Contains subsystems, OI devices, and commands.
*/
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    
    // Set default drive command
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    // Initialize drive to pose command for debugging
    SmartDashboard.putBoolean("Drive To Pose Enabled", false);
  }

   /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {
    
    elevator.setDefaultCommand(new RunCommand(() -> elevator.moveAmount(elevatorUpDown.getAsDouble()), elevator));
    //algaeArm.setDefaultCommand(new RunCommand(() -> algaeArm.moveTrigger(algaeArmTrigger.getAsDouble()), algaeArm)); // Updated to use moveTrigger
    shooterArm.setDefaultCommand(new RunCommand(() -> shooterArm.moveAmount(shooterArmUpDown.getAsDouble()), shooterArm));
    shooterPivot.setDefaultCommand(new RunCommand(() -> shooterPivot.moveAmount(shooterPivotUpDown.getAsDouble()), shooterPivot));

    //climber.setDefaultCommand(new RunCommand(() -> climber.moveAmount(elevatorUpDown.getAsDouble()), climber));

    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Updated default command for AlgaeShooter using suppliers
    algaeShooter.setDefaultCommand(new RunCommand(() -> {
        // Get trigger values from the suppliers
        double leftTrigger = algaeShooterIntake.getAsDouble();
        double rightTrigger = algaeShooterOutake.getAsDouble();
        
        // Control logic for the algae shooter based on triggers
        if (leftTrigger > AlgaeShooterConstants.triggerThreshold) {
            // Left trigger controls intake (forward) at variable speed
            double speed = leftTrigger * AlgaeShooterConstants.maxTriggerIntake;
            algaeShooter.setSpeed(speed);
        } 
        else if (rightTrigger > AlgaeShooterConstants.triggerThreshold) {
            // Right trigger controls outake (reverse) at variable speed
            double speed = rightTrigger * AlgaeShooterConstants.maxTriggerOutake;
            algaeShooter.setSpeed(speed);
        }
        else {
            // If both triggers are below threshold, stop the motor
            algaeShooter.setSpeed(0);
        }
    }, algaeShooter));
    

      buttonBox1.button(3).onTrue(new InstantCommand(() -> buttonBox.deleteFirstTarget()));
      buttonBox1.button(2).onTrue(new InstantCommand(() -> buttonBox.clearTargets()));
      buttonBox1.button(1).onTrue(new InstantCommand(() -> buttonBox.deleteLastTarget()));

      buttonBox1.button(9).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C400")));
      buttonBox1.button(9).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C401")));
      buttonBox1.button(9).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C410")));
      buttonBox1.button(9).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C411")));
      buttonBox1.button(9).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C420")));
      buttonBox1.button(9).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C421")));
      buttonBox1.button(9).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C430")));
      buttonBox1.button(9).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C431")));
      buttonBox1.button(10).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C300")));
      buttonBox1.button(10).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C301")));
      buttonBox1.button(10).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C310")));
      buttonBox1.button(10).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C311")));
      buttonBox1.button(10).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C320")));
      buttonBox1.button(10).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C321")));
      buttonBox1.button(10).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C330")));
      buttonBox1.button(10).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C331")));
      buttonBox1.button(11).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C200")));
      buttonBox1.button(11).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C201")));
      buttonBox1.button(11).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C210")));
      buttonBox1.button(11).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C211")));
      buttonBox1.button(11).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C220")));
      buttonBox1.button(11).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C221")));
      buttonBox1.button(11).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C230")));
      buttonBox1.button(11).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C231")));
      buttonBox1.button(6).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C100")));
      buttonBox1.button(6).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C101")));
      buttonBox1.button(6).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C110")));
      buttonBox1.button(6).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C111")));
      buttonBox1.button(6).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C120")));
      buttonBox1.button(6).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C121")));
      buttonBox1.button(6).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C130")));
      buttonBox1.button(6).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C131")));
      buttonBox1.button(7).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C600")));
      buttonBox1.button(7).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C601")));
      buttonBox1.button(7).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C610")));
      buttonBox1.button(7).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C611")));
      buttonBox1.button(7).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C620")));
      buttonBox1.button(7).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C621")));
      buttonBox1.button(7).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C630")));
      buttonBox1.button(7).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C631")));
      buttonBox1.button(8).and(buttonBox2.button(5)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C500")));
      buttonBox1.button(8).and(buttonBox2.button(1)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C501")));
      buttonBox1.button(8).and(buttonBox2.button(6)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C510")));
      buttonBox1.button(8).and(buttonBox2.button(2)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C511")));
      buttonBox1.button(8).and(buttonBox2.button(7)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C520")));
      buttonBox1.button(8).and(buttonBox2.button(3)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C521")));
      buttonBox1.button(8).and(buttonBox2.button(8)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C530")));
      buttonBox1.button(8).and(buttonBox2.button(4)).onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));

      
      buttonBox1.button(4).onTrue(new InstantCommand(() -> buttonBox.requeueLastTarget()));
      //buttonBox1.button(4).onTrue(new InstantCommand(() -> buttonBox.addTarget("SR")));

    driverXbox.rightBumper().whileTrue(CommandFactory.scoreBasedOnQueueCommandDriveAutoNOSHOOT(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this));
    driverXbox.leftBumper().onTrue(CommandFactory.setIntakeCommand(shooter, shooterArm, shooterPivot, elevator));
  
    driverXbox.x().onTrue(shooter.shooterIntakeCommand());
    driverXbox.x().onFalse(shooter.shooterZeroSpeedCommand());
    driverXbox.y().onTrue(shooter.shooterOutakeCommand());
    driverXbox.y().onFalse(shooter.shooterZeroSpeedCommand());


    driverXbox.a().onTrue(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, shooterPivot, elevator, buttonBox));
    driverXbox.b().onTrue(CommandFactory.setElevatorZero(shooter, shooterArm, shooterPivot, elevator));

    driverXbox.pov(0).onTrue(CommandFactory.pullOffHighBall(shooter, shooterArm, shooterPivot, elevator));
    driverXbox.pov(180).onTrue(CommandFactory.pullOffLowBall(shooter, shooterArm, shooterPivot, elevator));
    
    driverXbox.pov(90).onTrue(CommandFactory.setAlgaeIntakeCommand(algaeArm, algaeShooter));
    driverXbox.pov(270).onTrue(CommandFactory.algaeStowCommand(algaeArm, algaeShooter));





    
    // Hold back button to temporarily use drive-to-pose
    driverXbox.back().whileTrue(tempDriveToPoseCommand);
    
    // Toggle drive-to-pose with start button
    driverXbox.start().onTrue(
        Commands.either(
            Commands.runOnce(() -> tempDriveToPoseCommand.cancel()),
            Commands.runOnce(() -> tempDriveToPoseCommand.schedule()),
            () -> tempDriveToPoseCommand.isScheduled()
        )
    );
    

    // Cancel drive-to-pose when driver provides manual input
    driverXbox.axisMagnitudeGreaterThan(0, 0.1)
        .or(driverXbox.axisMagnitudeGreaterThan(1, .1))
        .or(driverXbox.axisMagnitudeGreaterThan(4, .1))
        .or(driverXbox.axisMagnitudeGreaterThan(5, .1))
        .onTrue(Commands.runOnce(() -> {
            drivebase.setCancel(true);
            if (tempDriveToPoseCommand.isScheduled()) {
                tempDriveToPoseCommand.cancel();
            }
        }));

    driverXbox.x().onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));
    driverXbox.y().onTrue(
        Commands.either(
            Commands.runOnce(() -> tempDriveToPoseCommand.cancel()),
            Commands.runOnce(() -> tempDriveToPoseCommand.schedule()),
            () -> tempDriveToPoseCommand.isScheduled()
        )
    );





    opXbox.pov(180).onTrue(CommandFactory.setClimbPosition(algaeArm, shooter, shooterArm, shooterPivot, elevator));
    opXbox.pov(90).onTrue(algaeArm.algaeArmStraightOutCommand());

    opXbox.a().onTrue(algaeShooter.algaeShooterIntakeCommand());
    opXbox.a().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());

    opXbox.b().onTrue(algaeShooter.algaeShooterOutakeCommand());
    opXbox.b().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());
    
    opXbox.x().onTrue(climber.climberExtendCommand());
    opXbox.x().onFalse(climber.climberStopCommand());

    opXbox.y().onTrue(climber.climberRetractCommand());
    opXbox.y().onFalse(climber.climberStopCommand());

    opXbox.rightBumper().onTrue(algaeArm.algaeArmGroundIntakeCommand());
    opXbox.leftBumper().onTrue(algaeArm.algaeArmStowUpCommand());

    opXbox.pov(0).onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));


    chooser.setDefaultOption("Right", rightAuto);
    chooser.addOption("Left", leftAuto);
    SmartDashboard.putData(chooser);

  }

  public Command changeDriveSpeedCommand(float speed)
  {
    return new InstantCommand(() -> targetDriveSpeed = speed);
  }

  public void setDriveSpeedBasedOnElevatorAndCloseness()
  {    
    // Base speed determined by elevator position using constants


    targetDriveSpeed = elevator.isRaisedEnough() ? SpeedConstants.elevatorRaisedSpeed : SpeedConstants.elevatorLoweredSpeed;

    // Update zone statuses
    Pose2d currentPose = drivebase.getPose();
    isInReefZone = isInReefZone(currentPose);

    isInCoralStationLeftZone = isInCoralStationLeft(currentPose);
    isInCoralStationRightZone = isInCoralStationRight(currentPose);

    /*
    // Reset distance to a large value by default
    distanceToTarget = Double.POSITIVE_INFINITY;
    

    
    // Adjust speed based on proximity to target if there's a valid target
    TargetClass currentTarget = buttonBox.peekNextTarget();
    if (currentTarget != null) {
      // Get current robot position and target position
      Pose2d robotPose = drivebase.getPose();
      Pose2d targetPoseNonCorrected = new Pose2d(currentTarget.getX(), currentTarget.getY(), 
                                    new Rotation2d(currentTarget.getZ()));

      Pose2d targetPose = TargetClass.toPose2d(targetPoseNonCorrected);
      // Calculate and store distance to target
      distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      
      
      // Define local proximity checks that always use current distance value using constants
      isVeryClose = distanceToTarget < SpeedConstants.veryCloseDistance;
      isClose = distanceToTarget < SpeedConstants.closeDistance;
      isApproaching = distanceToTarget < SpeedConstants.approachingDistance;
      isLinedUp = distanceToTarget < SpeedConstants.linedUpDistance;
      
      // Graduated speed reduction based on distance using constants
      if (isVeryClose) {
        targetDriveSpeed = Math.min(baseSpeed, SpeedConstants.veryCloseSpeedFactor);
      } else if (isClose) {
        targetDriveSpeed = Math.min(baseSpeed, SpeedConstants.closeSpeedFactor);
      } else if (isApproaching) {
        targetDriveSpeed = Math.min(baseSpeed, SpeedConstants.approachingSpeedFactor);
      } else {
        targetDriveSpeed = baseSpeed; // Use the base speed from elevator status
      }
      

      // For debugging
      SmartDashboard.putNumber("Distance to Target", distanceToTarget);
      SmartDashboard.putBoolean("Very Close to Target", isVeryClose);
      SmartDashboard.putBoolean("Close to Target", isClose);
      SmartDashboard.putBoolean("Approaching Target", isApproaching);
    } else {
      // No target or not in drive-to-pose mode, just use elevator-based speed
      targetDriveSpeed = baseSpeed;
      
      // Reset the dashboard indicators when not targeting
      SmartDashboard.putNumber("Distance to Target", distanceToTarget);
      SmartDashboard.putBoolean("Very Close to Target", false);
      SmartDashboard.putBoolean("Close to Target", false);
      SmartDashboard.putBoolean("Approaching Target", false);
    }
*/
    // Apply zone-based speed modifier
    float zoneModifier = getZoneSpeedMultiplier();
    
    targetDriveSpeed = Math.min(targetDriveSpeed, targetDriveSpeed * zoneModifier);

    // Smooth the speed transition
    smoothDriveSpeed();

    SmartDashboard.putNumber("Target Drive Speed", targetDriveSpeed);
    SmartDashboard.putNumber("Actual Drive Speed", actualDriveSpeed);
    SmartDashboard.putNumber("Drive Speed", targetDriveSpeed);
    SmartDashboard.putNumber("Zone Modifier", zoneModifier);
    
    // Update zone status on dashboard
    SmartDashboard.putBoolean("In Reef Zone", isInReefZone);
    SmartDashboard.putBoolean("In Coral Station Left", isInCoralStationLeftZone);  // Renamed
    SmartDashboard.putBoolean("In Coral Station Right", isInCoralStationRightZone); // Renamed

    driveY = () -> -driverXbox.getLeftY() * targetDriveSpeed;
    driveX = () -> -driverXbox.getLeftX() * targetDriveSpeed;

    angSpeed = () -> (-driverXbox.getRightX() * .8) * targetDriveSpeed;

    // If drive-to-pose is active, update target position status
    if (tempDriveToPoseCommand.isScheduled() && isDriveToPoseEnabled) {
      TargetClass target = buttonBox.peekNextTarget();
      
      if (target != null) {
          // Get current robot pose and target pose
          Pose2d robotPose = drivebase.getPose();
          Pose2d targetPose = new Pose2d(target.getX(), target.getY(), new Rotation2d(target.getZ()));
          Pose2d allianceAdjustedPose = TargetClass.toPose2d(targetPose);
          
          // Calculate position and rotation errors
          double positionError = robotPose.getTranslation().getDistance(allianceAdjustedPose.getTranslation());
          double rotationError = Math.abs(robotPose.getRotation().minus(allianceAdjustedPose.getRotation()).getRadians());
          rotationError = Math.abs(Math.atan2(Math.sin(rotationError), Math.cos(rotationError))); // Normalize to [0, π]
          
          // Update status based on errors
          isAtTargetPosition = positionError <= positionTolerance;
          isAtTargetRotation = rotationError <= rotationTolerance;
          isAtTarget = isAtTargetPosition && isAtTargetRotation;
          
          // Display on dashboard
          SmartDashboard.putNumber("Position Error", positionError);
          SmartDashboard.putNumber("Rotation Error (deg)", Units.radiansToDegrees(rotationError));
          SmartDashboard.putBoolean("At Target Position", isAtTargetPosition);
          SmartDashboard.putBoolean("At Target Rotation", isAtTargetRotation);
          SmartDashboard.putNumber("Rotation Error (deg)", Units.radiansToDegrees(rotationError));
          SmartDashboard.putBoolean("At Target Position", isAtTargetPosition);
          SmartDashboard.putBoolean("At Target Rotation", isAtTargetRotation);
          SmartDashboard.putBoolean("At Target", isAtTarget);
          
          // Update drive status message
          if (isAtTarget) {
              SmartDashboard.putString("Drive Status", "Target Reached!");
          } else if (isAtTargetPosition) {
              SmartDashboard.putString("Drive Status", "Position Reached - Aligning Rotation");
          } else if (isAtTargetRotation) {
              SmartDashboard.putString("Drive Status", "Rotation Aligned - Moving to Position");
          } else {
              SmartDashboard.putString("Drive Status", "Driving to Target");
          }
      }
      
      // ...existing code...
  }

    // If drive-to-pose is active, update the PID controllers periodically
    if (tempDriveToPoseCommand.isScheduled() && isDriveToPoseEnabled) {
      // Periodically update controllers (e.g., every 5 frames)
      if (periodicCounter % 5 == 0) {
          pidControllersNeedUpdate = true;
          // Force the suppliers to generate new controllers
          driveToPoseXControllerSupplier.get();
          driveToPoseRotControllerSupplier.get();
          // Reset flag after update
          pidControllersNeedUpdate = false;
      }
  }

    // Update SmartDashboard with drive-to-pose status
    SmartDashboard.putBoolean("Drive To Pose Enabled", isDriveToPoseEnabled);
    SmartDashboard.putBoolean("Drive To Pose Active", isDriveToPoseActive());
  }

  /**
   * Check if robot is inside the reef zone (circle)
   */
  private boolean isInReefZone(Pose2d robotPose) {
    // Get alliance-relative reef center
    Pose2d reefCenter = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.reefCenterX, 
        ZoneConstants.reefCenterY, 
        new Rotation2d(0)));
    
    // Calculate distance from center of circle
    double distance = Math.sqrt(
        Math.pow(robotPose.getX() - reefCenter.getX(), 2) + 
        Math.pow(robotPose.getY() - reefCenter.getY(), 2));
    
    // Inside if distance is less than radius
    return distance < ZoneConstants.reefZoneRadius;
  }

  /**
   * Check if robot is in left coral station
   */
  private boolean isInCoralStationLeft(Pose2d robotPose) {
    // Get alliance-relative coordinates for coral station left
    Pose2d minCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.LCoralStationMinX, 
        ZoneConstants.LCoralStationMinY,
        new Rotation2d(0)));
    
    Pose2d maxCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.LCoralStationMaxX, 
        ZoneConstants.LCoralStationMaxY,
        new Rotation2d(0)));
    
    return isInRectangularZone(robotPose, 
        Math.min(minCorner.getX(), maxCorner.getX()),
        Math.max(minCorner.getX(), maxCorner.getX()),
        Math.min(minCorner.getY(), maxCorner.getY()),
        Math.max(minCorner.getY(), maxCorner.getY()));
  }

  /**
   * Check if robot is in right coral station
   */
  private boolean isInCoralStationRight(Pose2d robotPose) {
    // Get alliance-relative coordinates for coral station right
    Pose2d minCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.RCoralStationMinX, 
        ZoneConstants.RCoralStationMinY,
        new Rotation2d(0)));
    
    Pose2d maxCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.RCoralStationMaxX, 
        ZoneConstants.RCoralStationMaxY,
        new Rotation2d(0)));
    
    return isInRectangularZone(robotPose, 
        Math.min(minCorner.getX(), maxCorner.getX()),
        Math.max(minCorner.getX(), maxCorner.getX()),
        Math.min(minCorner.getY(), maxCorner.getY()),
        Math.max(minCorner.getY(), maxCorner.getY()));
  }

  /**
   * Helper method to check if a point is inside a rectangle
   */
  private boolean isInRectangularZone(Pose2d pose, double minX, double maxX, double minY, double maxY) {
    return (pose.getX() >= minX && pose.getX() <= maxX && 
            pose.getY() >= minY && pose.getY() <= maxY);
  }

  /**
   * Get speed multiplier based on what zone the robot is in
   */
  private float getZoneSpeedMultiplier() {
    if (isInReefZone) {
      return ZoneConstants.reefSpeedMultiplier;
    } else if (isInCoralStationLeftZone || isInCoralStationRightZone) {
      return ZoneConstants.coralStationMultiplier;
    }
    return 1.0f; // No speed reduction if not in any special zone
  }

  /**
   * Apply smooth interpolation to drive speed changes to prevent jerky movement
   */
  private void smoothDriveSpeed() {
    // Calculate the difference between target and actual speed
    float speedDifference = targetDriveSpeed - actualDriveSpeed;
    
    // Apply the smoothing factor to gradually adjust the actual speed
    actualDriveSpeed += speedDifference * ZoneConstants.speedSmoothingFactor;
    
    // Set the drive speed to the smoothed value
    targetDriveSpeed = actualDriveSpeed;
  }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
      // An example command will be run in autonomous
      return chooser.getSelected();
    }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Get dynamic translation constraints based on distance to target
   */
  private Constraints getTranslationConstraintsForDistance(double distance) {
    // Use default "far" constraints if no target or infinite distance
    if (Double.isInfinite(distance) || distance <= 0) {
      // Use moderate default values that allow movement but not too fast
      return new Constraints(
          Constants.DriveToPoseConstants.FAR_MAX_VEL * 0.5,
          Constants.DriveToPoseConstants.FAR_MAX_ACCEL * 0.5);
    }
    else if (distance < Constants.DriveToPoseConstants.VERY_CLOSE_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.VERY_CLOSE_MAX_VEL,
          Constants.DriveToPoseConstants.VERY_CLOSE_MAX_ACCEL);
    } else if (distance < Constants.DriveToPoseConstants.CLOSE_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.CLOSE_MAX_VEL,
          Constants.DriveToPoseConstants.CLOSE_MAX_ACCEL);
    } else if (distance < Constants.DriveToPoseConstants.MID_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.MID_MAX_VEL,
          Constants.DriveToPoseConstants.MID_MAX_ACCEL);
    } else {
      return new Constraints(
          Constants.DriveToPoseConstants.FAR_MAX_VEL,
          Constants.DriveToPoseConstants.FAR_MAX_ACCEL);
    }
  }
  
  private Constraints getRotationConstraintsForDistance(double distance) {
    // Use default "far" constraints if no target or infinite distance
    if (Double.isInfinite(distance) || distance <= 0) {
      // Use moderate default values that allow rotation but not too fast
      return new Constraints(
          Constants.DriveToPoseConstants.FAR_MAX_ROT_VEL * 0.5,
          Constants.DriveToPoseConstants.FAR_MAX_ROT_ACCEL * 0.5);
    }
    else if (distance < Constants.DriveToPoseConstants.VERY_CLOSE_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.VERY_CLOSE_MAX_ROT_VEL,
          Constants.DriveToPoseConstants.VERY_CLOSE_MAX_ROT_ACCEL);
    } else if (distance < Constants.DriveToPoseConstants.CLOSE_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.CLOSE_MAX_ROT_VEL,
          Constants.DriveToPoseConstants.CLOSE_MAX_ROT_ACCEL);
    } else if (distance < Constants.DriveToPoseConstants.MID_DISTANCE) {
      return new Constraints(
          Constants.DriveToPoseConstants.MID_MAX_ROT_VEL,
          Constants.DriveToPoseConstants.MID_MAX_ROT_ACCEL);
    } else {
      return new Constraints(
          Constants.DriveToPoseConstants.FAR_MAX_ROT_VEL,
          Constants.DriveToPoseConstants.FAR_MAX_ROT_ACCEL);
    }
  }
  
  // Add a method to check if drive-to-pose is active
  public boolean isDriveToPoseActive() {
    return tempDriveToPoseCommand.isScheduled();
  }
  
  // Counter for periodic updates
  private int periodicCounter = 0;
  
  // Make sure this gets called periodically
  public void periodic() {
    periodicCounter++;
  }
  
  // Method to check if the robot has reached the target
  public boolean isAtTargetPosition() {
    return isAtTargetPosition;
  }
  
  public boolean isAtTargetRotation() {
    return isAtTargetRotation;
  }
  
  public boolean isAtTarget() {
    return isAtTarget;
  }
  
  // Allow setting custom tolerances if needed
  public void setPositionTolerance(double meters) {
    positionTolerance = meters;
  }
  
  public void setRotationTolerance(double radians) {
    rotationTolerance = radians;
  }
  
  // Reset to default from constants
  public void resetTolerancesToDefault() {
    positionTolerance = Constants.DriveToPoseConstants.POSITION_TOLERANCE;
    rotationTolerance = Constants.DriveToPoseConstants.ROTATION_TOLERANCE;
  }
}