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

    public static Command setIntakeCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .alongWith(shooterPivot.setCenterCommand())
      .andThen(shooter.shooterIntakeCommand())
      .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(elevator.setElevatorPickupPlusCommand())
      .andThen(new WaitCommand(1))
      .andThen(shooter.shooterZeroSpeedCommand());


      command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

      return command;
  }

  public static Command setIntakeManualCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = elevator.setElevatorPickupCommand()
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmLoadCommand())
    .alongWith(shooterPivot.setCenterCommand())
    .andThen(shooter.shooterIntakeCommand())
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand());


    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
}
/*
public static Command setAlgaeIntakeCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, AlgaeShooter algaeShooter, ShooterPivot shooterPivot, Elevator elevator) {
      
  Command command  = algaeArm.algaeArmGroundIntakeCommand()
  .andThen(algaeShooter.algaeShooterIntakeCommand())
  
  .andThen(new WaitUntilCommand(algaeShooter.))
  .andThen(elevator.setElevatorPickupPlusCommand())
  .andThen(new WaitCommand(1))
  .andThen(shooter.shooterZeroSpeedCommand());


  command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

  return command;
}  
*/

  public static Command pullOffHighBall(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
  }
  
  public static Command pullOffLowBall(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmBallCommand());

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
  }

  
  public static Command setElevatorZero(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand() //Will make this straight up at some point
    .alongWith(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
      .andThen(elevator.setfullElevatorRetractCommand())
      .andThen(algaeArm.algaeArmStowUpCommand());


    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
}

public static Command setClimbPositionsCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
  Command command  = elevator.setElevatorClimbPoseCommand()
  .andThen(new WaitUntilCommand(elevator.isClearToClimbAngle()))
  .andThen(shooterArm.shooterArmClimbCommand())
  .alongWith(shooterPivot.setCenterCommand());

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
}

/*
    public static Command scoreL2Command(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator){

      Command command  = shooterArm.shooterArmScoreLOWCommand()
      .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
      .andThen(elevator.setElevatorL2Command())
      .alongWith(shooterPivot.setLeftInitialCommand());

        command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

        return command;
    }

    public static Command scoreL3Command(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator){
        
        Command command  = shooterArm.shooterArmScoreLOWCommand()
        .alongWith(elevator.setElevatorL3Command())
        .alongWith(shooterPivot.setLeftInitialCommand());
  
          command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
  
          return command;

    }

    public static Command scoreL4Command(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator){
        
        Command command  = shooterArm.shooterArmScoreHIGHCommand()
        .alongWith(elevator.setElevatorL4Command())
        .alongWith(shooterPivot.setLeftInitialCommand());
  
          command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
  
          return command;
    }
    */
/*
        public static Command scoreTest(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

          Command command = shooterArm.shooterArmScoreLOWCommand()
            .andThen(new InstantCommand (() -> buttonBox.addTarget("C310")))
            .andThen(new WaitUntilCommand(buttonBox.isSlow()))
            .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
            .andThen(elevator.setElevatorL3Command())
            .andThen(shooterPivot.setRightInitalCommand())
            .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
            .andThen(new WaitUntilCommand(buttonBox.isClose()))
            .andThen(shooter.shooterOutakeCommand())
            .andThen(new WaitCommand(.5))
            .andThen(shooter.shooterZeroSpeedCommand())
            .andThen(buttonBox.getNextTargetCommand());

            command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

        return command; 
    } 

    public static Command scoreTestL4(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

      Command command = shooterArm.shooterArmScoreHIGHCommand()
        .andThen(new InstantCommand (() -> buttonBox.addTarget("C310")))
        .andThen(new WaitUntilCommand(buttonBox.isSlow()))
        .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
        .andThen(elevator.setElevatorL4Command())
        .andThen(shooterPivot.setRightInitalCommand())
        .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
        .andThen(new WaitUntilCommand(buttonBox.isClose()))
        .andThen(shooter.shooterOutakeCommand())
        .andThen(new WaitCommand(.5))
        .andThen(shooter.shooterZeroSpeedCommand())
        .andThen(buttonBox.getNextTargetCommand());

        command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command; 
} 
*/

public static Command scoreBasedOnQueueCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

  Command command = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
    //.andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    //.andThen(new WaitUntilCommand(buttonBox.isClose()))
    //.andThen(shooter.shooterOutakeCommand())
    //.andThen(new WaitCommand(.5))
    //.andThen(shooter.shooterZeroSpeedCommand())
    //.andThen(robotContainer.stopButtonBoxFollowCommand())
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.driveToPose(buttonBox)
  .alongWith(CommandFactory.scoreBasedOnQueueCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.isLinedUpTrigger()))
  .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}
public static Command scoreBasedOnQueueCommandDriveAuto(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.driveToPose(buttonBox)
    .alongWith(CommandFactory.scoreBasedOnQueueCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox))
    .andThen(new WaitUntilCommand(robotContainer.isLinedUpTrigger()))
    
    .andThen(shooter.shooterIntakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterOutakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand())
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}


public static Command sourceDrive(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer){

  Command command = setIntakeCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(new WaitUntilCommand(robotContainer.isVeryCloseTrigger()))
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
    .andThen(shooter.shooterZeroSpeedCommand());

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

return command; 
}

public static Command sourceDriveAuto(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase){

  Command command = drivebase.driveToPose(buttonBox)
    .alongWith(CommandFactory.setIntakeCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator));

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

  return command; 
}


/*
public static Command scoreBasedOnQueueCommandDriveTest(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

  Command command = shooter.shooterIntakeCommand()
    .andThen(new InstantCommand (() -> buttonBox.addTarget("C310")))
    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
    .andThen(shooter.shooterZeroSpeedCommand())
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
    .andThen(new WaitUntilCommand(buttonBox.isClose()))
    .andThen(shooter.shooterOutakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand())
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}
*/

public static Command DriveTest(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand();

  command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
  return command; 
}


public static Command LeftAutonCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C531"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(shooterPivot.setCenterCommand())
  .andThen(elevator.setElevatorPickupCommand())
  .andThen(new WaitCommand(0.5))

  // Improved sequence with better status checking
  .andThen(drivebase.pathfindThenFollowPath("Left To Source"))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(shooterPivot.setCenterCommand())
  .andThen(elevator.setElevatorPickupCommand())
  .andThen(new WaitCommand(0.5))

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
    
  command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
  return command; 
}

public static Command RightAutonCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C331"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))

    // Improved sequence with better status checking
    .andThen(drivebase.pathfindThenFollowPath("Right to Source"))
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C231")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))

    // Second source run
    .andThen(drivebase.pathfindThenFollowPath("Right to Source"))
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

    .andThen(new InstantCommand(() -> buttonBox.addTarget("C230")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(shooterPivot.setCenterCommand())
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitCommand(0.5))
    
    // Optional third run depending on time
    .andThen(drivebase.pathfindThenFollowPath("Right to Source"))
    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(algaeArm, shooter, shooterArm, shooterPivot, elevator, buttonBox, robotContainer, drivebase))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}
}