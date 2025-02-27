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

import swervelib.SwerveInputStream;


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

  private boolean isClose = false;
  private boolean isVeryClose = false;
  private boolean isApproaching = false;
  private boolean isLinedUp = false;


  // Track distance to target for proximity calculations
  private double distanceToTarget = Double.POSITIVE_INFINITY;
  
  /**
   * Check if the robot is close to the target pose from the ButtonBox
   * 
   * @return A Trigger that activates when robot is within error allowance of target
   */

  DoubleSupplier headingXAng = () -> -driverXbox.getRightX();

  DoubleSupplier driveX;
  DoubleSupplier driveY;
  DoubleSupplier headingX = () -> -driverXbox.getRightX();
  DoubleSupplier headingY = () -> -driverXbox.getRightY();

  DoubleSupplier elevatorUpDown = () -> opXbox.getRightY();
  DoubleSupplier algaeArmUpDown = () -> opXbox.getRightX();
  DoubleSupplier shooterArmUpDown = () -> opXbox.getLeftY();
  DoubleSupplier shooterPivotUpDown = () -> opXbox.getLeftX(); //Questionable Name Practices... Shooter Pivot UP DOWN not Left Right??

  DoubleSupplier climberUpDown = () -> opXbox.getRightX();
  

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

  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driverXbox.getLeftY() * -1,
  () -> driverXbox.getLeftX() * -1)
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
    
    public Command leftAuto = CommandFactory.LeftAutonCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this);
    public Command rightAuto = CommandFactory.RightAutonCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this);

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

    //buttonBox1.button(1).onTrue(elevator.setElevatorL1Command());
   // buttonBox1.button(2).onTrue(elevator.setElevatorL2Command());
   // buttonBox1.button(3).onTrue(elevator.setElevatorL3Command());
    //buttonBox1.button(4).onTrue(elevator.setElevatorL4Command());

    //buttonBox1.button(6).onTrue(algaeArm.algaeArmStowUpCommand());
    //buttonBox1.button(5).onTrue(algaeArm.algaeArmGroundIntakeCommand());

    
   // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
   // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngleKeyboard);
    
    //elevator.setDefaultCommand(new RunCommand(() -> elevator.moveAmount(elevatorUpDown.getAsDouble()), elevator));
    algaeArm.setDefaultCommand(new RunCommand(() -> algaeArm.moveAmount(algaeArmUpDown.getAsDouble()), algaeArm));
    shooterArm.setDefaultCommand(new RunCommand(() -> shooterArm.moveAmount(shooterArmUpDown.getAsDouble()), shooterArm));
    shooterPivot.setDefaultCommand(new RunCommand(() -> shooterPivot.moveAmount(shooterPivotUpDown.getAsDouble()), shooterPivot));

    //climber.setDefaultCommand(new RunCommand(() -> climber.moveAmount(elevatorUpDown.getAsDouble()), climber));

    /*if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);

    }
    */

    //driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      //Maincontrols here
     // this.shooterPivot.setDefaultCommand(new InstantCommand(() -> shooterPivot.moveAmount((float) opXbox.getRightX()), shooterPivot));

      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.a().onTrue(shooter.shooterIntakeCommand());
      driverXbox.a().onFalse(shooter.shooterZeroSpeedCommand());
      driverXbox.b().onTrue(shooter.shooterOutakeCommand());
      driverXbox.b().onFalse(shooter.shooterZeroSpeedCommand());

      //driverXbox.a().onTrue(algaeShooter.algaeShooterIntakeCommand());
      //driverXbox.a().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());
      //driverXbox.b().onTrue(algaeShooter.algaeShooterOutakeCommand());
      //driverXbox.b().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());
      

      
      //driverXbox.pov(0).onTrue(CommandFactory.scoreL4Command(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      //driverXbox.pov(90).onTrue(CommandFactory.scoreL3Command(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      //driverXbox.pov(180).onTrue(CommandFactory.scoreL2Command(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      driverXbox.pov(270).onTrue(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, this, drivebase));

      //opXbox.x().onTrue(shooterArm.shooterArmLoadCommand());
      //opXbox.y().onTrue(elevator.setElevatorPickupCommand());

      buttonBox1.button(1).onTrue(new InstantCommand(() -> buttonBox.deleteFirstTarget()));
      buttonBox1.button(2).onTrue(new InstantCommand(() -> buttonBox.clearTargets()));
      buttonBox1.button(3).onTrue(new InstantCommand(() -> buttonBox.deleteLastTarget()));

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

      
      buttonBox1.button(5).onTrue(new InstantCommand(() -> buttonBox.addTarget("SL")));
      buttonBox1.button(4).onTrue(new InstantCommand(() -> buttonBox.addTarget("SR")));

      driverXbox.rightBumper().onTrue(CommandFactory.scoreBasedOnQueueCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox));
      //temp
   // driverXbox.a().onTrue(new InstantCommand(() -> buttonBox.addTarget("C0000")));

    //driverXbox.y().onTrue(new InstantCommand(() -> drivebase.followPath("Right to 0")));

    //driverXbox.rightBumper().onTrue(new InstantCommand(() -> (CommandFactory.scoreBasedOnQueueCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox)));

    //buttonBox1.button(1).onTrue(CommandFactory.scoreTest(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox));
    //driverXbox.x().whileTrue(CommandFactory.scoreBasedOnQueueCommandDrive(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, this));

    driverXbox.y().onTrue(CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator));
    
    driverXbox.leftBumper().onTrue(CommandFactory.setIntakeManualCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator));

    opXbox.a().onTrue(algaeShooter.algaeShooterIntakeCommand());
    opXbox.a().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());

    opXbox.b().onTrue(algaeShooter.algaeShooterOutakeCommand());
    opXbox.b().onFalse(algaeShooter.algaeShooterZeroSpeedCommand());
    
    opXbox.x().onTrue(climber.climberExtendCommand());
    opXbox.x().onFalse(climber.climberStopCommand());

    opXbox.y().onTrue(climber.climberRetractCommand());
    opXbox.y().onFalse(climber.climberStopCommand());

    driverXbox.x().onTrue(drivebase.driveToPose(buttonBox));

    //driverXbox.b().onTrue(CommandFactory.scoreTestSim(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox));

      //drivebase.setDefaultCommand(driveButtonBoxInputCommand);
   
  // driverXbox.axisMagnitudeGreaterThan(0, 0.1).or(driverXbox.axisMagnitudeGreaterThan(1, .1)).onTrue(
    //   new InstantCommand(() -> {
         // On resume, simply schedule the followQueueCommand.
     //    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
     //  }, drivebase));

    //driverXbox.b().onTrue(new InstantCommand(() -> (drivebase.setDefaultCommand(driveButtonBoxInput))));

    // Now override this behavior by binding the resume action to the right bumper.
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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
    // Base speed determined by elevator position
    float baseSpeed = elevator.isRaised() ? 0.3f : 1.0f;
    
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
      
      // Define local proximity checks that always use current distance value
      isVeryClose = distanceToTarget < 0.5;
      isClose = distanceToTarget < 1.0;
      isApproaching = distanceToTarget < 1.5;
      isLinedUp = distanceToTarget < 0.08;
      
      
      // Graduated speed reduction based on distance
      if (isVeryClose) {
        driveSpeed = Math.min(baseSpeed, 0.2f);
      } else if (isClose) {
        driveSpeed = Math.min(baseSpeed, 0.2f); //For testing
      } else if (isApproaching) {
        driveSpeed = Math.min(baseSpeed, 0.2f); //For testing
      } else {
        driveSpeed = baseSpeed; // Use the base speed from elevator status
      }
      
      
      // For debugging
      SmartDashboard.putNumber("Distance to Target", distanceToTarget);
      SmartDashboard.putBoolean("Very Close to Target", isVeryClose);
      SmartDashboard.putBoolean("Close to Target", isClose);
      SmartDashboard.putBoolean("Approaching Target", isApproaching);
    } else {
      // No target or not in drive-to-pose mode, just use elevator-based speed
      driveSpeed = baseSpeed;
      
      // Reset the dashboard indicators when not targeting
      SmartDashboard.putNumber("Distance to Target", Double.POSITIVE_INFINITY);
      SmartDashboard.putBoolean("Very Close to Target", false);
      SmartDashboard.putBoolean("Close to Target", false);
      SmartDashboard.putBoolean("Approaching Target", false);
    }
    
    SmartDashboard.putNumber("Drive Speed", driveSpeed);
    
    //isApproachingSupplier = () -> isApproaching;
    //isVeryCloseSupplier = () -> isVeryClose;
    //isCloseSupplier = () -> isClose;
    //isLinedUpSupplier = () -> isLinedUp;

    driveX = () -> -driverXbox.getLeftX() * driveSpeed;
    driveY = () -> -driverXbox.getLeftY() * driveSpeed;

    isApproachingTrigger();
    isCloseTrigger();
    isVeryCloseTrigger();
    isLinedUpTrigger();
  }
  
  public Trigger isApproachingTrigger(){
    return new Trigger(() -> isApproaching);
  }

  public Trigger isCloseTrigger(){
    return new Trigger(() -> isClose);
  }

  public Trigger isVeryCloseTrigger(){
    return new Trigger(() -> isVeryClose);
  }
  public Trigger isLinedUpTrigger(){
    return new Trigger(() -> isLinedUp);
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