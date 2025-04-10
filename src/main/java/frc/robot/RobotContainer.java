// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
import frc.robot.subsystems.PaulServo;

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

  private boolean loadedSingleTime = false;

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
  private boolean isInBargeZone = false;

  // Add flag to track if we're currently being held outside reef zone
  private boolean heldOutsideReefZone = false;

  // Add trigger for reef zone restriction status
  public Trigger reefZoneRestrictionActiveTrigger() {
    return new Trigger(() -> heldOutsideReefZone);
  }

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

  //DoubleSupplier climberUpDown = () -> opXbox.getRightX();

  // Add suppliers for algae shooter triggers
  DoubleSupplier algaeShooterIntake = () -> driverXbox.getLeftTriggerAxis();
  DoubleSupplier algaeShooterOutake = () -> driverXbox.getRightTriggerAxis();

  // Add supplier for funnel control
  DoubleSupplier climberUpDown = () -> opXbox.getLeftTriggerAxis() - opXbox.getRightTriggerAxis();

  // Add flag for full speed toggle
  private boolean fullSpeedModeEnabled = false;

  // Add a boolean to track Quest nav state
  private boolean isUsingQuestRobotContainer = true;

  private boolean isUsingQuestToStart = false;

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
  
  private final LED led = new LED();

  
  private final PaulServo servo = new PaulServo();

  private final ButtonBox buttonBox = new ButtonBox(drivebase);
  

  // Add QuestNavVision instance
  private final frc.robot.subsystems.swervedrive.QuestNavVision questNavVision = new frc.robot.subsystems.swervedrive.QuestNavVision();

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

  public Trigger bargeZoneTrigger() {
    return new Trigger(() -> isInBargeZone);
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


  public Command leftAuto = CommandFactory.LeftAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase, this, funnel);

  public Command leftCenterAuto = CommandFactory.LeftCenterAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase, this, funnel);
  public Command rightCenterAuto = CommandFactory.RightCenterAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase, this, funnel);

  public Command rightAuto = CommandFactory.RightAutonCommand(shooter, shooterArm, elevator, buttonBox, drivebase, this, funnel);

  SendableChooser<Command> chooser = new SendableChooser<>();

  // Add tracking variables for autonomous pose initialization
  private String lastSelectedAuto = "";
  private boolean poseInitialized = false;
  private Optional<DriverStation.Alliance> lastAlliance = Optional.empty(); // Track last alliance color

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the Funnel-AlgaeArm safety connection
    funnel.setAlgaeArmReference(algaeArm);
    
    // Set RobotContainer reference for the shooter
    shooter.setRobotContainer(this);
    
    // Configure the trigger bindings
    configureBindings();

    //UsbCamera camera = CameraServer.startAutomaticCapture();'
    
    
    //camera.setResolution(240, 240);
    //camera.setFPS(20);

    // Initial pose setup based on default selection
    initializeRobotPoseForAuto();
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

    opXbox.axisMagnitudeGreaterThan(5, 0.2).whileTrue(new RunCommand(() -> elevator.moveAmount(elevatorUpDown.getAsDouble()), elevator));

    opXbox.axisMagnitudeGreaterThan(1, 0.2).whileTrue(new RunCommand(() -> shooterArm.moveAmount(shooterArmUpDown.getAsDouble()), shooterArm));

    climber.setDefaultCommand(new RunCommand(() -> climber.moveWithPower(climberUpDown), climber));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);


    driverXbox.axisMagnitudeGreaterThan(2, 0.2).or(driverXbox.axisMagnitudeGreaterThan(3, 0.2))
    .whileTrue(new RunCommand(() -> {
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
    }, algaeShooter))
    .onFalse(new InstantCommand(() -> algaeShooter.setSpeed(0), algaeShooter));




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
    driverXbox.back().onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    driverXbox.start().onTrue(toggleFullSpeedModeCommand());

    //driverXbox.rightBumper().onTrue(CommandFactory.scoreBasedOnQueueCommandDriveAutoNOSHOOT(shooter, shooterArm, elevator, buttonBox, drivebase, this));

    driverXbox.rightBumper().whileTrue(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    driverXbox.leftBumper().onTrue(CommandFactory.setIntakeCommand(shooter, shooterArm, elevator, funnel, algaeArm, algaeShooter, this, led));

    driverXbox.rightBumper().whileTrue(drivebase.driveToPoseProfiled(buttonBox));



    driverXbox.x().onTrue(shooter.shooterIntakeCommand());
    driverXbox.x().onFalse(shooter.shooterZeroSpeedCommand());
    
    //driverXbox.x().onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));

    driverXbox.y().onTrue(shooter.shooterOutakeCommand());
    driverXbox.y().and(shooter.L1ScoringTrigger()).onTrue(CommandFactory.finishL1ScoreCommand(shooter, shooterArm, elevator, algaeArm, algaeShooter, funnel));
    driverXbox.y().whileTrue(led.runPattern("MANUAL_SHOOTING_PATTERN").repeatedly());
    driverXbox.y().onFalse(shooter.shooterZeroSpeedCommand().alongWith(led.setAlliancePattern()));

    driverXbox.a().onTrue(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    driverXbox.b().onTrue(CommandFactory.setElevatorZero(shooter, shooterArm, elevator));

    driverXbox.pov(0).onTrue(CommandFactory.pullOffHighAboveBall(shooter, shooterArm, elevator));
    driverXbox.pov(180).onTrue(CommandFactory.scoreL1CommandNOSHOOT(shooter, shooterArm, elevator, algaeArm, algaeShooter, funnel));
    
    driverXbox.rightStick().onTrue(CommandFactory.ballDown(shooter, shooterArm, elevator));
    driverXbox.leftStick().onTrue(CommandFactory.pullOffLowBall(shooter, shooterArm, elevator));

    driverXbox.pov(90).onTrue(CommandFactory.setAlgaeIntakeCommand(algaeArm, algaeShooter));
    driverXbox.pov(270).onTrue(CommandFactory.algaeStowCommand(algaeArm, algaeShooter));


    opXbox.pov(180).onTrue(CommandFactory.setClimbPositionNoArm(algaeArm, funnel, climber));

    opXbox.start().onTrue(CommandFactory.setClimbPositionArmOnly(elevator, shooterArm));

    opXbox.pov(90).onTrue(algaeArm.algaeArmHoldCommand());


    opXbox.a().onTrue(servo.setEngageCommand());

    opXbox.b().onTrue(servo.setDisengageCommand());
    
    // Add new position control commands
    // Fully retract climber (for stowing)
    opXbox.pov(270).onTrue(climber.climberFullRetractCommand());

    //opXbox.rightBumper().onTrue(algaeArm.algaeArmGroundIntakeCommand());
    //opXbox.leftBumper().onTrue(algaeArm.algaeArmStowUpCommand());

    opXbox.rightBumper().whileTrue(new RunCommand(() -> algaeArm.moveAmount(1), algaeArm));
    opXbox.leftBumper().whileTrue(new RunCommand(() -> algaeArm.moveAmount(-1), algaeArm));

    opXbox.x().whileTrue(new RunCommand(() -> funnel.moveAmount(1), funnel));
    opXbox.y().whileTrue(new RunCommand(() -> funnel.moveAmount(-1), funnel));

    //opXbox.start().onTrue(new InstantCommand(() -> drivebase.oldCameraMode(true)));
    //opXbox.back().onTrue(new InstantCommand(() -> drivebase.oldCameraMode(false)));

    opXbox.pov(0).onTrue(new InstantCommand(() -> buttonBox.addTarget("C531")));

    opXbox.back().onTrue(Commands.runOnce(() -> {
      // Toggle the state
      isUsingQuestRobotContainer = !isUsingQuestRobotContainer;
      
      // Apply the appropriate command based on new state
      if (isUsingQuestRobotContainer) {
        drivebase.setIsUsingQuest(true);
        SmartDashboard.putString("QuestNav InUSE", "QuestNav ENABLED");
      } else {
        drivebase.setIsUsingQuest(false);
        SmartDashboard.putString("QuestNav InUSE", "QuestNav DISABLED");
      }
    }));


    chooser.addOption("Left", leftAuto);
    chooser.addOption("Left Center", leftCenterAuto);
    chooser.addOption("Right Center", rightCenterAuto);
    chooser.setDefaultOption("Right", rightAuto);
     
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
      isLinedUp = distance <= 0.03 &&
          angleDifference <= Units.degreesToRadians(3.0);

      // If we just became lined up, clear the target visualization
      if (isLinedUp && !wasLinedUp) {
        drivebase.clearTargetVisualization();
        // Optional: provide haptic feedback or logging

        loadedSingleTime = false;

        // Run a more noticeable flash pattern and log it
        led.flashLEDs(Color.kGreen, 5).schedule();
        SmartDashboard.putString("LED Action", "Flashed GREEN for lineup");

        driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        // Schedule a command to stop rumble after a short duration
        Commands.waitSeconds(0.5)
            .andThen(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0))
            .schedule();
      } else if (!isLinedUp && !wasLinedUp) {
        // Only visualize the target if we're not lined up

        drivebase.visualizeTargetPose(allianceRelativeTarget);
      }

      if(loadedSingleTime == false){
        loadedSingleTime = shooter.coralLoadedTrigger().getAsBoolean();
      }

      // If coral is loaded and we're approaching the target, update LED pattern based on distance
      if (loadedSingleTime && currentTarget != null) {
        led.runDistanceBasedPatternWhenLoaded(distance);
      }
    } else {
      // No target, all triggers are false
      isApproaching = false;
      isClose = false;
      isVeryClose = false;
      isLinedUp = false;

      loadedSingleTime = false;

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
    isInBargeZone = isInBargeZone(currentPose);

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
    bargeZoneTrigger();

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
  
  private boolean isInBargeZone(Pose2d robotPose) {
    // Get alliance-relative coordinates for barge zone
    Pose2d minCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.BargeMinX,
        ZoneConstants.BargeMinY,
        new Rotation2d(0)));

    Pose2d maxCorner = TargetClass.toPose2d(new Pose2d(
        ZoneConstants.BargeMaxX,
        ZoneConstants.BargeMaxY,
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
    else if (isInBargeZone) {
      return ZoneConstants.bargeMultiplier;
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
   * Checks the current autonomous selection and sets the robot's initial pose accordingly.
   * Only updates the pose if the selection has changed, alliance has changed, or pose hasn't been initialized yet.
   * Will not perform any operations if the DriverStation is not connected.
   */
  public void initializeRobotPoseForAuto() {
    // First check if DriverStation is connected before proceeding
    if (!DriverStation.isDSAttached()) {
      // Don't modify robot pose if DriverStation is not connected
      SmartDashboard.putString("Auto Pose Status", "Waiting for DriverStation connection");
      return;
    }
    
    // Check if alliance color changed
    Optional<DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();
    boolean allianceChanged = !currentAlliance.equals(lastAlliance);
    
    if (allianceChanged) {
      // If alliance changed, force pose reinitialization
      poseInitialized = false;
      SmartDashboard.putString("Alliance Changed", 
          "From " + (lastAlliance.isPresent() ? lastAlliance.get().toString() : "UNKNOWN") + 
          " to " + (currentAlliance.isPresent() ? currentAlliance.get().toString() : "UNKNOWN"));
      // Update last alliance
      lastAlliance = currentAlliance;
    }
    
    Command selectedCommand = chooser.getSelected();
    if (selectedCommand == null) {
      SmartDashboard.putString("Auto Pose Status", "No autonomous selected");
      return;
    }
    
    String currentSelection;
    
    // Determine which auto is selected
    if (selectedCommand == leftAuto) {
      currentSelection = "Left";
    } else if (selectedCommand == leftCenterAuto) {
      currentSelection = "LeftCenter";
    } else if (selectedCommand == rightCenterAuto) {
      currentSelection = "RightCenter";
    } else if (selectedCommand == rightAuto) {
      currentSelection = "Right";
    } else {
      currentSelection = "Unknown";
    }
    
    // Only set the pose if the selection changed, alliance changed, or pose hasn't been initialized yet
    if (!currentSelection.equals(lastSelectedAuto) || !poseInitialized) {
      // Rest of the method remains the same
      if (currentSelection.equals("Left")) {
        // Set pose for Left autonomous using constants
        Pose2d leftStartPose = new Pose2d(
            Constants.TargetClassConstants.LeftStartX,
            Constants.TargetClassConstants.LeftStartY,
            new Rotation2d(Constants.TargetClassConstants.LeftStartZ));
        
        // Convert to alliance-relative coordinates
        Pose2d allianceRelativeLeftPose = TargetClass.toPose2d(leftStartPose);
        drivebase.resetOdometry(allianceRelativeLeftPose);
        
        SmartDashboard.putString("Auto Pose Initialized", "Left Start Position");
      } else if (currentSelection.equals("LeftCenter")) {
        // Set pose for Left Center autonomous
        Pose2d leftCenterStartPose = new Pose2d(
            Constants.TargetClassConstants.CenterStartX,
            Constants.TargetClassConstants.CenterStartY,
            new Rotation2d(Constants.TargetClassConstants.CenterStartZ));
        
        // Convert to alliance-relative coordinates
        Pose2d allianceRelativeLeftCenterPose = TargetClass.toPose2d(leftCenterStartPose);
        drivebase.resetOdometry(allianceRelativeLeftCenterPose);
        
        SmartDashboard.putString("Auto Pose Initialized", "Left Center Start Position");
      } else if (currentSelection.equals("RightCenter")) {
        // Set pose for Right Center autonomous
        Pose2d rightCenterStartPose = new Pose2d(
            Constants.TargetClassConstants.CenterStartX,
            Constants.TargetClassConstants.CenterStartY,
            new Rotation2d(Constants.TargetClassConstants.CenterStartZ));
        
        // Convert to alliance-relative coordinates
        Pose2d allianceRelativeRightCenterPose = TargetClass.toPose2d(rightCenterStartPose);
        drivebase.resetOdometry(allianceRelativeRightCenterPose);
        
        SmartDashboard.putString("Auto Pose Initialized", "Right Center Start Position");
      } else {
        // Default to Right autonomous pose
        Pose2d rightStartPose = new Pose2d(
            Constants.TargetClassConstants.RightStartX,
            Constants.TargetClassConstants.RightStartY,
            new Rotation2d(Constants.TargetClassConstants.RightStartZ));
        
        // Convert to alliance-relative coordinates
        Pose2d allianceRelativeRightPose = TargetClass.toPose2d(rightStartPose);
        drivebase.resetOdometry(allianceRelativeRightPose);
        
        SmartDashboard.putString("Auto Pose Initialized", "Right Start Position");
      }
      

      drivebase.setIsUsingQuest(false);

      // Add vision measurement cycling and Quest nav reset
      SmartDashboard.putString("QuestNav Status", "Cycling vision measurements...");
      
      // Cycle vision measurement 200 times to ensure stable odometry
      for (int i = 0; i < 200; i++) {
        drivebase.addVisionMeasurementCommand();
      }
      
      

      // After vision measurements are cycled, reset the QuestNav with current pose
      Pose2d currentPose = drivebase.getPose();

      questNavVision.setPose(currentPose);
      

      

      isUsingQuestToStart = SmartDashboard.getBoolean("isUsingQuestToStart", false);

      SmartDashboard.putBoolean("isUsingQuestToStart", isUsingQuestToStart);

      if(isUsingQuestToStart){
        SmartDashboard.putString("QuestNav InUSE", "QuestNav ENABLED");
      drivebase.setIsUsingQuest(true);
    }
    else{
      SmartDashboard.putString("QuestNav InUSE", "QuestNav DISABLED");
      drivebase.setIsUsingQuest(false);
    }


      SmartDashboard.putString("QuestNav Status", "Pose reset complete");

      // Update tracking variables
      lastSelectedAuto = currentSelection;
      poseInitialized = true;
      
      // Update status dashboard
      SmartDashboard.putString("Auto Pose Status", "Successfully initialized");
    }
  }
  
  // Add method to reset pose initialization flag (call this when a new match starts)
  public void resetPoseInitialization() {
    poseInitialized = false;
    lastSelectedAuto = "";
    lastAlliance = Optional.empty(); // Reset alliance tracking as well
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Ensure pose is initialized before returning autonomous command
    initializeRobotPoseForAuto();
    // Return the selected autonomous command
    return chooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}