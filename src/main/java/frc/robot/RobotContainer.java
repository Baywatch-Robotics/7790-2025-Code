// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
  public float driveSpeed = 0;
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

  // Track distance to target for proximity calculations
  private double distanceToTarget = Double.POSITIVE_INFINITY;
  
  /**
   * Check if the robot is close to the target pose from the ButtonBox
   * 
   * @return A Trigger that activates when robot is within error allowance of target
   */

  DoubleSupplier headingXAng = () -> -driverXbox.getRightX() * .8;

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

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driveY.getAsDouble(),
  () -> driveX.getAsDouble())
.withControllerRotationAxis(headingXAng)
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


  
    public Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    public Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngle);
    public Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    //public Command driveButtonBoxInputCommand = drivebase.driveFieldOriented(driveAngularVelocityDriveToPose);
    
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

    driverXbox.axisMagnitudeGreaterThan(0,0.1).or(driverXbox.axisMagnitudeGreaterThan(1, .1)).or(driverXbox.axisMagnitudeGreaterThan(4,.1)).or(driverXbox.axisMagnitudeGreaterThan(5, .1)).onTrue(new InstantCommand(() -> drivebase.setCancel(true)));

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
    return new InstantCommand(() -> driveSpeed = speed);
  }

  public void setDriveSpeedBasedOnElevatorAndCloseness()
  {    
    // Base speed determined by elevator position using constants
    float baseSpeed = elevator.isRaisedEnough() ? 
      SpeedConstants.elevatorRaisedSpeed : SpeedConstants.elevatorLoweredSpeed;

    // Update zone statuses
    Pose2d currentPose = drivebase.getPose();
    isInReefZone = isInReefZone(currentPose);
    isInCoralStationLeftZone = isInCoralStationLeft(currentPose);
    isInCoralStationRightZone = isInCoralStationRight(currentPose);
    
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

    // Apply zone-based speed modifier
    float zoneModifier = getZoneSpeedMultiplier();
    targetDriveSpeed *= zoneModifier;

    // Smooth the speed transition
    smoothDriveSpeed();

    SmartDashboard.putNumber("Target Drive Speed", targetDriveSpeed);
    SmartDashboard.putNumber("Actual Drive Speed", actualDriveSpeed);
    SmartDashboard.putNumber("Drive Speed", driveSpeed);
    SmartDashboard.putNumber("Zone Modifier", zoneModifier);
    
    // Update zone status on dashboard
    SmartDashboard.putBoolean("In Reef Zone", isInReefZone);
    SmartDashboard.putBoolean("In Coral Station Left", isInCoralStationLeftZone);  // Renamed
    SmartDashboard.putBoolean("In Coral Station Right", isInCoralStationRightZone); // Renamed

    driveY = () -> -driverXbox.getLeftY() * driveSpeed;
    driveX = () -> -driverXbox.getLeftX() * driveSpeed;
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
    driveSpeed = actualDriveSpeed;
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
}