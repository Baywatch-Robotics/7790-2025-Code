// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.SpeedConstants;
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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Elastic;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LED;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

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

  // Add constants for proximity thresholds
  // private static final double APPROACHING_DISTANCE_THRESHOLD = 2.0; // meters
  // private static final double CLOSE_DISTANCE_THRESHOLD = 1.0; // meters
  // private static final double VERY_CLOSE_DISTANCE_THRESHOLD = 0.3; // meters
  // private static final double LINED_UP_ANGLE_THRESHOLD = Math.toRadians(5.0);
  // // radians
  // private static final double LINED_UP_POSITION_THRESHOLD = 0.1; // meters

  // Add zone status tracking
  private boolean isInReefZone = false;
  private boolean isInCoralStationLeftZone = false; // Consistent naming
  private boolean isInCoralStationRightZone = false; // Consistent naming

  // Add flag to track if we're currently being held outside reef zone
  private boolean heldOutsideReefZone = false;

  // Add trigger for reef zone restriction status
  public Trigger reefZoneRestrictionActiveTrigger() {
    return new Trigger(() -> heldOutsideReefZone);
  }

  Supplier<ProfiledPIDController> xProfiledPID;
  Supplier<ProfiledPIDController> rotProfiledPID;

  DoubleSupplier headingXAng = () -> -driverXbox.getRightX() * .8;
  DoubleSupplier angSpeed;

  DoubleSupplier driveX;
  DoubleSupplier driveY;
  DoubleSupplier headingX = () -> -driverXbox.getRightX();
  DoubleSupplier headingY = () -> -driverXbox.getRightY();

  DoubleSupplier elevatorUpDown = () -> opXbox.getRightY();
  // DoubleSupplier algaeArmTriggerUp = () -> opXbox.getLeftTriggerAxis();
  // DoubleSupplier algaeArmTriggerDown = () -> opXbox.getLeftTriggerAxis();
  DoubleSupplier shooterArmUpDown = () -> opXbox.getLeftY();
  DoubleSupplier shooterPivotUpDown = () -> opXbox.getLeftX(); // Questionable Name Practices... Shooter Pivot UP DOWN
                                                               // not Left Right??

  DoubleSupplier climberUpDown = () -> opXbox.getRightX();

  // Add suppliers for algae shooter triggers
  DoubleSupplier algaeShooterIntake = () -> driverXbox.getLeftTriggerAxis();
  DoubleSupplier algaeShooterOutake = () -> driverXbox.getRightTriggerAxis();

  // Add supplier for funnel control
  //DoubleSupplier funnelUpDown = () -> opXbox.getLeftTriggerAxis() - opXbox.getRightTriggerAxis();

  // Add flag for full speed toggle
  private boolean fullSpeedModeEnabled = false;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final AlgaeArm algaeArm = new AlgaeArm();
  private final AlgaeShooter algaeShooter = new AlgaeShooter();
  private final Shooter shooter = new Shooter();
  private final ShooterArm shooterArm = new ShooterArm();
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator(this);

  // Initialize funnel subsystem
  private final Funnel funnel = new Funnel();
  
  private final LED LED = new LED();

  private final ButtonBox buttonBox = new ButtonBox(drivebase);

  // Triggers for proximity detection
  public Trigger approachingTrigger() {
    return new Trigger(() -> isApproaching);
  }

  public Trigger closeTrigger() {
    return new Trigger(() -> isClose);
  }

  public Trigger veryCloseTrigger() {
    return new Trigger(() -> isVeryClose);
  }

  public Trigger linedUpTrigger() {
    return new Trigger(() -> isLinedUp);
  }

  // Triggers for zone detection
  public Trigger reefZoneTrigger() {
    return new Trigger(() -> isInReefZone);
  }

  public Trigger coralStationLeftTrigger() {
    return new Trigger(() -> isInCoralStationLeftZone);
  }

  public Trigger coralStationRightTrigger() {
    return new Trigger(() -> isInCoralStationRightZone);
  }

  public Trigger anyZoneTrigger() {
    return new Trigger(() -> isInReefZone || isInCoralStationLeftZone || isInCoralStationRightZone);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driveY.getAsDouble(),
      () -> driveX.getAsDouble())
      .withControllerRotationAxis(() -> angSpeed.getAsDouble())
      .deadband(Constants.DEADBAND)
      .scaleTranslation(1)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(headingX,
      headingY).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
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
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  // Create drive commands
  public Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  public Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  // public Command driveFieldOrientedDriveToPose =
  // drivebase.driveFieldOriented(driveToPoseStream);

  public Command leftAuto = CommandFactory.LeftAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase, this);
  public Command rightAuto = CommandFactory.RightAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase,
      this);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */

  private void configureBindings() {

    elevator.setDefaultCommand(new RunCommand(() -> elevator.moveAmount(elevatorUpDown.getAsDouble()), elevator));
    // algaeArm.setDefaultCommand(new RunCommand(() ->
    // algaeArm.moveTrigger(algaeArmTrigger.getAsDouble()), algaeArm)); // Updated
    // to use moveTrigger
    shooterArm
        .setDefaultCommand(new RunCommand(() -> shooterArm.moveAmount(shooterArmUpDown.getAsDouble()), shooterArm));

    // climber.setDefaultCommand(new RunCommand(() ->
    // climber.moveAmount(elevatorUpDown.getAsDouble()), climber));

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
      } else if (rightTrigger > AlgaeShooterConstants.triggerThreshold) {
        // Right trigger controls outake (reverse) at variable speed
        double speed = rightTrigger * AlgaeShooterConstants.maxTriggerOutake;
        algaeShooter.setSpeed(speed);
      } else {
        // If both triggers are below threshold, stop the motor
        algaeShooter.setSpeed(0);
      }
    }, algaeShooter));

    // Add default command for funnel manual control
    //funnel.setDefaultCommand(new RunCommand(() -> funnel.moveAmount(funnelUpDown.getAsDouble()), funnel));

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
    // buttonBox1.button(4).onTrue(new InstantCommand(() ->
    // buttonBox.addTarget("SR")));

    // Modified: combine zero gyro with full speed toggle
    // driverXbox.back().onTrue(
    // new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    // driverXbox.start().onTrue(toggleFullSpeedModeCommand());

    driverXbox.rightBumper().whileTrue(CommandFactory.scoreBasedOnQueueCommandDriveAutoNOSHOOT(shooter, shooterArm,
        elevator, buttonBox, drivebase, this));
    driverXbox.leftBumper().onTrue(CommandFactory.setIntakeCommand(shooter, shooterArm, elevator));

    driverXbox.x().onTrue(shooter.shooterIntakeCommand());
    driverXbox.x().onFalse(shooter.shooterZeroSpeedCommand());
    driverXbox.y().onTrue(shooter.shooterOutakeCommand());
    driverXbox.y().onFalse(shooter.shooterZeroSpeedCommand());

    driverXbox.a().onTrue(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    driverXbox.b().onTrue(CommandFactory.setElevatorZero(shooter, shooterArm, elevator));

    driverXbox.pov(0).onTrue(CommandFactory.pullOffHighBall(shooter, shooterArm, elevator));
    driverXbox.pov(180).onTrue(CommandFactory.pullOffLowBall(shooter, shooterArm, elevator));

    driverXbox.pov(90).onTrue(CommandFactory.setAlgaeIntakeCommand(algaeArm, algaeShooter));
    driverXbox.pov(270).onTrue(CommandFactory.algaeStowCommand(algaeArm, algaeShooter));

    // Hold back button to temporarily use drive-to-pose
    // driverXbox.back().whileTrue(tempDriveToPoseCommand);

    // Toggle drive-to-pose with start button
    driverXbox.back().onTrue(drivebase.driveToPose(buttonBox));

    driverXbox.start().onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));

    // Cancel drive-to-pose when driver provides manual input
    driverXbox.axisMagnitudeGreaterThan(0, 0.1)
        .or(driverXbox.axisMagnitudeGreaterThan(1, .1))
        .or(driverXbox.axisMagnitudeGreaterThan(4, .1))
        .or(driverXbox.axisMagnitudeGreaterThan(5, .1))
        .onTrue(Commands.runOnce(() -> drivebase.setCancel(true)));

    // driverXbox.x().onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));
    // driverXbox.y().onTrue(
    // Commands.either(
    // Commands.runOnce(() -> tempDriveToPoseCommand.cancel()),
    // Commands.runOnce(() -> tempDriveToPoseCommand.schedule()),
    // () -> tempDriveToPoseCommand.isScheduled()
    // )
    // );

    opXbox.pov(180).onTrue(CommandFactory.setClimbPosition(algaeArm, shooter, shooterArm, elevator));
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

  /**
   * Command to toggle full speed mode on/off
   */
  public Command toggleFullSpeedModeCommand() {
    return Commands.runOnce(() -> {
      fullSpeedModeEnabled = !fullSpeedModeEnabled;
      SmartDashboard.putBoolean("Full Speed Mode", fullSpeedModeEnabled);
      Elastic.sendNotification(
          new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO,
              "Speed Mode Changed",
              "Full speed mode " + (fullSpeedModeEnabled ? "enabled" : "disabled")));
    });
  }

  public Command changeDriveSpeedCommand(float speed) {
    return new InstantCommand(() -> targetDriveSpeed = speed);
  }

  /**
   * Update proximity status triggers based on current robot position and target
   * pose
   */
  public void updateProximityStatus() {
    // Get current robot pose
    Pose2d currentPose = drivebase.getPose();

    // Get the current target pose from the ButtonBox, if available
    TargetClass currentTarget = buttonBox.currentTargetClassSupplier.get();

    if (currentTarget != null) {
      // Convert target to Pose2d
      Pose2d targetPose = new Pose2d(
          currentTarget.getX(),
          currentTarget.getY(),
          Rotation2d.fromRadians(currentTarget.getZ()));

      // Convert to alliance-relative coordinates
      Pose2d allianceRelativeTarget = TargetClass.toPose2d(targetPose);

      // Calculate distance to target
      double distance = Math.sqrt(
          Math.pow(currentPose.getX() - allianceRelativeTarget.getX(), 2) +
              Math.pow(currentPose.getY() - allianceRelativeTarget.getY(), 2));

      // Calculate angle difference (absolute)
      double angleDifference = Math.abs(
          currentPose.getRotation().getRadians() -
              allianceRelativeTarget.getRotation().getRadians());
      // Normalize angle to -π to π
      angleDifference = angleDifference > Math.PI ? (2 * Math.PI) - angleDifference : angleDifference;

      // Update trigger states based on distance - now using constants from Constants
      // class
      isApproaching = distance <= DriveToPoseConstants.APPROACHING_DISTANCE_THRESHOLD;
      isClose = distance <= DriveToPoseConstants.CLOSE_DISTANCE_THRESHOLD;
      isVeryClose = distance <= DriveToPoseConstants.VERY_CLOSE_DISTANCE_THRESHOLD;

      // Check if lined up (both position and rotation are within thresholds)
      boolean wasLinedUp = isLinedUp; // Store previous state to detect changes
      isLinedUp = distance <= DriveToPoseConstants.LINED_UP_POSITION_THRESHOLD &&
          angleDifference <= DriveToPoseConstants.LINED_UP_ANGLE_THRESHOLD;

      // If we just became lined up, clear the target visualization
      if (isLinedUp && !wasLinedUp) {
        drivebase.clearTargetVisualization();
        // Optional: provide haptic feedback or logging
        driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        // Schedule a command to stop rumble after a short duration
        Commands.waitSeconds(0.5)
            .andThen(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0))
            .schedule();
      } else if (!isLinedUp && !wasLinedUp) {
        // Only visualize the target if we're not lined up

        drivebase.visualizeTargetPose(allianceRelativeTarget);
      }
    } else {
      // No target, all triggers are false
      isApproaching = false;
      isClose = false;
      isVeryClose = false;
      isLinedUp = false;

      // Clear target visualization
      drivebase.clearTargetVisualization();
    }

    // Log lined up status to dashboard for debugging
    SmartDashboard.putBoolean("Is Lined Up", isLinedUp);

    // Update SmartDashboard with proximity status
    SmartDashboard.putBoolean("Approaching Target", isApproaching);
    SmartDashboard.putBoolean("Close to Target", isClose);
    SmartDashboard.putBoolean("Very Close to Target", isVeryClose);
    SmartDashboard.putBoolean("Lined Up with Target", isLinedUp);

    // Add distance info if there's a target
    if (currentTarget != null) {
      Pose2d currentPoseForDashboard = drivebase.getPose();
      Pose2d targetPoseForDashboard = TargetClass.toPose2d(new Pose2d(
          currentTarget.getX(),
          currentTarget.getY(),
          Rotation2d.fromRadians(currentTarget.getZ())));

      // Display distance to target on dashboard
      double distanceToDashboard = Math.sqrt(
          Math.pow(currentPoseForDashboard.getX() - targetPoseForDashboard.getX(), 2) +
              Math.pow(currentPoseForDashboard.getY() - targetPoseForDashboard.getY(), 2));
      SmartDashboard.putNumber("Distance to Target", distanceToDashboard);
    } else {
      SmartDashboard.putNumber("Distance to Target", -1); // No target
    }
  }

  public void setDriveSpeedBasedOnElevatorAndCloseness() {
    // Update proximity status
    updateProximityStatus();

    // Check if the elevator is at intake position - if so, use full speed
    // regardless of height
    if (elevator.isAtIntakePosition()) {
      targetDriveSpeed = SpeedConstants.intakePositionSpeed;
      SmartDashboard.putBoolean("At Intake Position", true);
    } else {
      SmartDashboard.putBoolean("At Intake Position", false);

      // Determine drive speed based on elevator height category using the new
      // constants
      Elevator.ElevatorHeight heightCategory = elevator.getElevatorHeightCategory();

      // ALWAYS apply elevator-based speed restrictions, regardless of full speed mode
      switch (heightCategory) {
        case FULLY_RAISED:
          targetDriveSpeed = SpeedConstants.elevatorFullyRaisedSpeed;
          break;
        case MID_RAISED:
          targetDriveSpeed = SpeedConstants.elevatorMidRaisedSpeed;
          break;
        case PARTIALLY_RAISED:
          targetDriveSpeed = SpeedConstants.elevatorPartiallyRaisedSpeed;
          break;
        case LOWERED:
        default:
          targetDriveSpeed = SpeedConstants.elevatorLoweredSpeed;
          break;
      }

    }

    // Update SmartDashboard with current elevator height state
    SmartDashboard.putString("Elevator Height State", elevator.getElevatorHeightCategory().toString());

    // Update zone statuses
    Pose2d currentPose = drivebase.getPose();
    isInReefZone = isInReefZone(currentPose);
    isInCoralStationLeftZone = isInCoralStationLeft(currentPose);
    isInCoralStationRightZone = isInCoralStationRight(currentPose);

    // Apply zone-based speed modifier ONLY if full speed mode is disabled
    if (!fullSpeedModeEnabled) {
      float zoneModifier = getZoneSpeedMultiplier();
      targetDriveSpeed = Math.min(targetDriveSpeed, zoneModifier);
      SmartDashboard.putNumber("Zone Modifier", zoneModifier);
    } else {
      // When full speed mode is enabled, no zone modifier is applied
      SmartDashboard.putNumber("Zone Modifier", 1.0);
    }

    // Update SmartDashboard with full speed mode status
    SmartDashboard.putBoolean("Full Speed Mode", fullSpeedModeEnabled);

    reefZoneTrigger();
    coralStationLeftTrigger();
    coralStationRightTrigger();

    // Smooth the speed transition
    smoothDriveSpeed();

    // Update dashboard with speed values
    SmartDashboard.putNumber("Target Drive Speed", targetDriveSpeed);
    SmartDashboard.putNumber("Actual Drive Speed", actualDriveSpeed);
    SmartDashboard.putNumber("Drive Speed", targetDriveSpeed);

    // Update zone status on dashboard
    SmartDashboard.putBoolean("In Reef Zone", isInReefZone);
    SmartDashboard.putBoolean("In Coral Station Left", isInCoralStationLeftZone);
    SmartDashboard.putBoolean("In Coral Station Right", isInCoralStationRightZone);

    // Update drive suppliers with new speed
    driveY = () -> -driverXbox.getLeftY() * targetDriveSpeed;
    driveX = () -> -driverXbox.getLeftX() * targetDriveSpeed;
    angSpeed = () -> -driverXbox.getRightX() * targetDriveSpeed;
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

  public Command setControllerRumbleCommand(double intensity) {
    return new InstantCommand(() -> {
      driverXbox.setRumble(RumbleType.kBothRumble, intensity);
    });
  }

  public Command stopControllerRumbleCommand() {
    return new InstantCommand(() -> {
      driverXbox.setRumble(RumbleType.kBothRumble, 0);
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the selected autonomous command
    return chooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}