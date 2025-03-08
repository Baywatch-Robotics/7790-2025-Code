package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
      //.andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(elevator.setElevatorPickupPlusCommand())
      .andThen(new WaitCommand(5))
      .andThen(shooter.shooterZeroSpeedCommand());


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
    //.andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }
  
  public static Command pullOffLowBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    //.andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  


public static Command scoreBasedOnQueueCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox)
  .andThen(Commands.runOnce(() -> robotContainer.tempDriveToPoseCommand.schedule()))
  .andThen(new WaitUntilCommand(robotContainer.targetReachedTrigger()));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox)
  .andThen(Commands.runOnce(() -> robotContainer.tempDriveToPoseCommand.schedule()))
  .andThen(new WaitUntilCommand(robotContainer.closeTrigger()))
  .andThen(shooter.shooterIntakeCommand())
  .andThen(new WaitUntilCommand(robotContainer.targetReachedTrigger()))
  
  .andThen(shooter.shooterOutakeCommand())
    
  .andThen(new WaitCommand(.5))
  .andThen(shooter.shooterZeroSpeedCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}


public static Command sourceDrive(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer){

  Command command = setIntakeCommand(shooter, shooterArm, elevator)
    .andThen(new WaitUntilCommand(robotContainer.coralStationLeftTrigger().or(robotContainer.coralStationRightTrigger())))
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
    .andThen(shooter.shooterZeroSpeedCommand());

    command.addRequirements(shooter, shooterArm, elevator);

return command; 
}


public static Command sourceDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase){

  Command command = Commands.runOnce(() -> robotContainer.tempDriveToPoseCommand.schedule())
  .andThen(new WaitCommand(1))
  .andThen(CommandFactory.setIntakeCommand(shooter, shooterArm, elevator));

    command.addRequirements(shooter, shooterArm, elevator);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C521"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C620")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C621")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand());
    
  command.addRequirements(shooter, shooterArm, elevator);
  return command; 
}

public static Command RightAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C321"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand())
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand())
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C221")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand())
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand())
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C220")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand())
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(robotContainer.disableDriveToPoseCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}
}