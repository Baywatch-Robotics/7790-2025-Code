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
import frc.robot.subsystems.Coral.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CommandFactory {

    public static Command setIntakeCommand(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .alongWith(shooterPivot.setCenterCommand())
      .andThen(shooter.shooterIntakeCommand())
      .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(elevator.setElevatorPickupPlusCommand())
      .andThen(new WaitCommand(1))
      .andThen(shooter.shooterZeroSpeedCommand());


      command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

      return command;
  }

  public static Command setElevatorZero(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand() //Will make this straight up at some point
    .alongWith(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
      .andThen(elevator.setfullElevatorRetractCommand());


    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

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

  public static Command setClimbPosition(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator){

    Command command = CommandFactory.setElevatorZero(shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitCommand(1))
    .andThen(algaeArm.algaeArmStraightOutCommand())
    .andThen(shooterPivot.setLeftInitialCommand());


    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
  }

  public static Command pullOffHighBall(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    //.andThen(elevator.setElevatorHighBallCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

    return command;
  }
  
  public static Command pullOffLowBall(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    //.andThen(elevator.setElevatorLowBallCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

    return command;
  }

  


public static Command scoreBasedOnQueueCommand(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterPivot.setCenterCommand())
    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = //CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, shooterPivot, elevator, buttonBox)
  robotContainer.enableDriveToPoseCommand()
  .andThen(new WaitUntilCommand(robotContainer.targetReachedTrigger()));
    
    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAuto(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = //CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, shooterPivot, elevator, buttonBox)
  robotContainer.enableDriveToPoseCommand()
  .andThen(new WaitUntilCommand(robotContainer.targetReachedTrigger()))
    
  
    .andThen(shooter.shooterIntakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterOutakeCommand());
    /*
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand());
    */
    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);
    return command; 
}


public static Command sourceDrive(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer){

  Command command = setIntakeCommand(shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitUntilCommand(robotContainer.coralStationLeftTrigger().or(robotContainer.coralStationRightTrigger())))
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
    .andThen(shooter.shooterZeroSpeedCommand());

    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

return command; 
}


public static Command sourceDriveAuto(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase){

  Command command = robotContainer.enableDriveToPoseCommand()
  .andThen(new WaitCommand(2));
    //.alongWith(CommandFactory.setIntakeCommand(shooter, shooterArm, shooterPivot, elevator));

    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C531"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(shooterPivot.setCenterCommand())
  .andThen(elevator.setElevatorPickupCommand())
  .andThen(new WaitCommand(0.5))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(robotContainer.disableDriveToPoseCommand())
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(shooterPivot.setCenterCommand())
  .andThen(elevator.setElevatorPickupCommand());
  /*
  // Second source run
  .andThen(drivebase.pathfindThenFollowPath("Left To Source"))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  // Score at position 6 again
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C631")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(shooterPivot.setCenterCommand())
  .andThen(elevator.setElevatorPickupCommand())
  .andThen(new WaitCommand(0.5))
  
  // Optional third run depending on time
  .andThen(drivebase.pathfindThenFollowPath("Left To Source"))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  */
  command.addRequirements(shooter, shooterArm, shooterPivot, elevator);
  return command; 
}

public static Command RightAutonCommand(Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C331"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))

    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    

    .andThen(new InstantCommand(() -> buttonBox.addTarget("C231")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))

    
    // Second source run
    .andThen(drivebase.pathfindThenFollowPath("Right to Source"))
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

    .andThen(new InstantCommand(() -> buttonBox.addTarget("C230")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))
    
    // Optional third run depending on time
    .andThen(drivebase.pathfindThenFollowPath("Right to Source"))
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    
    command.addRequirements(shooter, shooterArm, shooterPivot, elevator);
    return command; 
}
}