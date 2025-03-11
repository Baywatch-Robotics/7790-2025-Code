package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Algae.AlgaeShooter;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CommandFactory {

   
    public static Command setIntakeCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .andThen(shooter.shooterIntakeCommand())
      .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(shooterArm.shooterArmOutLoadCommand())
      .andThen(elevator.setElevatorPickupPlusCommand())
      .andThen(new WaitCommand(.5))
      .andThen(shooter.shooterZeroSpeedCommand());


      command.addRequirements(shooter, shooterArm, elevator);

      return command;
  }
  

  public static Command setIntakeCommandFORAUTOONLY(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
    

    Command command  = elevator.setElevatorPickupCommand()
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmLoadCommand())
    .andThen(shooter.shooterIntakeCommand())
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()));


    command.addRequirements(shooter, shooterArm, elevator);

    return command;
}

  public static Command setElevatorZero(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand() //Will make this straight up at some point
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
      .andThen(elevator.setfullElevatorRetractCommand());


    command.addRequirements(shooter, shooterArm, elevator);

    return command;
}

public static Command setAlgaeIntakeCommand(AlgaeArm algaeArm, AlgaeShooter algaeShooter) {
    Command command = algaeArm.algaeArmGroundIntakeCommand()
        .andThen(algaeShooter.algaeShooterIntakeCommand())
        .andThen(new WaitUntilCommand(algaeShooter.algaeLoadedTrigger()))
        .andThen(algaeArm.algaeArmHoldCommand())
        .andThen(new WaitCommand(2))
        .andThen(algaeShooter.algaeShooterZeroSpeedCommand());

    command.addRequirements(algaeArm, algaeShooter);
    return command;
}

public static Command algaeStowCommand(AlgaeArm algaeArm, AlgaeShooter algaeShooter) {
  
    Command command = algaeArm.algaeArmStowUpCommand()
        .andThen(algaeShooter.algaeShooterZeroSpeedCommand());

    command.addRequirements(algaeArm, algaeShooter);
    return command;
}

  public static Command setClimbPosition(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, Elevator elevator){

    Command command = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorClimbPoseCommand())
    .andThen(new WaitCommand(1))
    .andThen(algaeArm.algaeArmStraightOutCommand());


    command.addRequirements(algaeArm, shooter, shooterArm, elevator);

    return command;
  }

  public static Command pullOffHighBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }
  
  public static Command pullOffLowBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  
public static Command scoreBasedOnQueueCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmBasedOnQueueCommand(buttonBox)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox)
  .andThen(drivebase.startDriveToPose(buttonBox, elevator))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRST(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(new WaitUntilCommand(robotContainer.approachingTrigger()))
  .andThen(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(shooter.shooterOutakeCommand())
  .andThen(new WaitCommand(.5))
  .andThen(shooter.shooterZeroSpeedCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(new WaitUntilCommand(robotContainer.approachingTrigger()))
  .andThen(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(shooter.shooterOutakeCommand())
  .andThen(new WaitCommand(.5))
  .andThen(shooter.shooterZeroSpeedCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command sourceDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(new WaitCommand(1.5))
  .andThen(CommandFactory.setIntakeCommandFORAUTOONLY(shooter, shooterArm, elevator));

    command.addRequirements(shooter, shooterArm, elevator);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C530"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C631")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(shooter, shooterArm, elevator);
  return command; 
}

public static Command RightAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C331"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C130")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C131")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}
}