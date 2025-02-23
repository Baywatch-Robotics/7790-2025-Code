package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.Coral.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class CommandFactory {

    public static Command setIntakeCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .alongWith(shooterPivot.setCenterCommand())
      .andThen(shooter.shooterIntakeCommand());


      command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

      return command;
  }

  public static Command setElevatorZero(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand() //Will make this straight up at some point
    .alongWith(shooterPivot.setCenterCommand())
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
      .andThen(elevator.setfullElevatorRetractCommand());


    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command;
}

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

 
    public static Command sourceRightDrive(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

      Command command = setIntakeCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator)
        .andThen(new InstantCommand (() -> buttonBox.addTarget("SR")))
        .andThen(new WaitUntilCommand(buttonBox.isClose()))
        .andThen(shooter.shooterZeroSpeedCommand());

        command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

    return command; 
} 
public static Command sourceLeftDrive(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox){

  Command command = setIntakeCommand(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(new InstantCommand (() -> buttonBox.addTarget("SL")))
    .andThen(new WaitUntilCommand(buttonBox.isClose()))
    .andThen(shooter.shooterZeroSpeedCommand());

    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);

return command; 
}



public static Command scoreBasedOnQueueCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer){

  Command command = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
.andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
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
public static Command scoreBasedOnQueueCommandDrive(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer){

  Command command = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
.andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(new WaitUntilCommand(buttonBox.isClose()))
    .andThen(shooter.shooterOutakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand())
    .andThen(robotContainer.stopButtonBoxFollowCommand())
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}
}