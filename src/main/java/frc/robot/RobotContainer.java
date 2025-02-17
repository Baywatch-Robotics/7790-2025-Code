// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Algae.AlgaeShooter;
import frc.robot.subsystems.Coral.Scope;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.Coral.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.AprilTagVision;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
  
  DoubleSupplier headingXAng = () -> -driverXbox.getRightX();

  DoubleSupplier driveX = () -> driverXbox.getLeftX();
  DoubleSupplier driveY = () -> driverXbox.getLeftY();
  DoubleSupplier headingX = () -> driverXbox.getRightX();
  DoubleSupplier headingY = () -> driverXbox.getRightY();

  DoubleSupplier elevatorUpDown = () -> opXbox.getRightY();
  DoubleSupplier algaeArmUpDown = () -> opXbox.getRightX();
  DoubleSupplier shooterArmUpDown = () -> opXbox.getLeftY();
  DoubleSupplier shooterPivotUpDown = () -> opXbox.getLeftX(); //Questionable Name Practices... Shooter Pivot UP DOWN not Left Right??
  //DoubleSupplier climberUpDown = () -> opXbox.getLeftY();
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  
  
  private final AlgaeArm algaeArm = new AlgaeArm();
  private final AlgaeShooter algaeShooter = new AlgaeShooter();
  //private final Scope scope = new Scope();
  private final Shooter shooter = new Shooter();
  private final ShooterArm shooterArm = new ShooterArm();
  private final ShooterPivot shooterPivot = new ShooterPivot();
  //private final AprilTagVision aprilTagVision = new AprilTagVision();
  //private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator();

  //private final Funnel funnel = new Funnel();
  private final LED LED = new LED();
  private final ButtonBox buttonBox = new ButtonBox(drivebase);

  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driverXbox.getLeftY() * .4,
  () -> driverXbox.getLeftX() * .4)
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

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    Command driveButtonBoxInputCommand = drivebase.driveFieldOriented(driveButtonBoxInput);
   // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
   // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngleKeyboard);
    
    elevator.setDefaultCommand(new RunCommand(() -> elevator.moveAmount(elevatorUpDown.getAsDouble()), elevator));
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
      opXbox.a().onTrue(shooter.shooterIntakeCommand());
      opXbox.a().onFalse(shooter.shooterZeroSpeedCommand());
      opXbox.b().onTrue(shooter.shooterOutakeCommand());
      opXbox.b().onFalse(shooter.shooterZeroSpeedCommand());

      opXbox.y().onTrue(CommandFactory.setIntakeCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      opXbox.x().onTrue(CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      opXbox.leftBumper().onTrue(CommandFactory.scoreL2Command(algaeArm, shooter, shooterArm, shooterPivot, elevator));
      opXbox.rightBumper().onTrue(CommandFactory.scoreL3Command(algaeArm, shooter, shooterArm, shooterPivot, elevator));

      //opXbox.x().onTrue(shooterArm.shooterArmLoadCommand());
      //opXbox.y().onTrue(elevator.setElevatorPickupCommand());

      

      buttonBox1.button(1).and(buttonBox1.button(2))
      .onTrue(new InstantCommand(() -> buttonBox.addTarget("C0000")));

    buttonBox2.button(9)
      .onTrue(new InstantCommand(() -> buttonBox.addTarget("SL")));

      //temp
    driverXbox.a().onTrue(new InstantCommand(() -> buttonBox.addTarget("C0000")));

    //driverXbox.y().onTrue(new InstantCommand(() -> drivebase.followPath("Right to 0")));


    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //driverXbox.axisMagnitudeGreaterThan(0, 0.1).or(driverXbox.axisMagnitudeGreaterThan(1, .1)).onTrue(driveFieldOrientedDirectAngle);

   // driverXbox.axisMagnitudeGreaterThan(0, 0.1).or(driverXbox.axisMagnitudeGreaterThan(1, .1)).onTrue(driveFieldOrientedDirectAngleKeyboard);
   
  // driverXbox.axisMagnitudeGreaterThan(0, 0.1).or(driverXbox.axisMagnitudeGreaterThan(1, .1)).onTrue(
    //   new InstantCommand(() -> {
         // On resume, simply schedule the followQueueCommand.
     //    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
     //  }, drivebase));

    // Create a command that continuously follows the queue (if present) unless overridden.

    // Now override this behavior by binding the resume action to the right bumper.
  //  driverXbox.b().onTrue(
     //   new InstantCommand(() -> {
          // On resume, simply schedule the followQueueCommand.
    //      drivebase.setDefaultCommand(driveButtonBoxInputCommand);
    //    }, drivebase));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}