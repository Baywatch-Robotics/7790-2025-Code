package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Algae.AlgaeShooter;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Climber;

public class CommandFactory {

   
    public static Command setIntakeCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter, RobotContainer robotContainer, LED led) {
      
      // Run the LED pattern first as a separate command
      Command ledCommand = led.runPattern("INTAKE_PATTERN");
      
      Command mainCommand = funnel.funnelHomeCommand()
      // Remove the LED command from here since we'll combine it at the end
      .andThen(algaeArm.algaeArmStowUpCommand())
      .andThen(algaeShooter.algaeShooterZeroSpeedCommand())
      .andThen(shooterArm.shooterArmScoreLOWCommand().onlyIf(robotContainer.reefZoneTrigger().and(shooterArm.isClearToElevate())))
      .andThen(elevator.setElevatorPickupCommand())
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      // Only proceed to loading position when outside reef zone
      .andThen(new WaitUntilCommand(robotContainer.reefZoneTrigger().negate()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .andThen(shooter.shooterIntakeCommand())
      .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(shooterArm.shooterArmOutLoadCommand())
      .andThen(elevator.setElevatorPickupPlusCommand())
      .andThen(new WaitCommand(.5))
      .andThen(shooter.shooterZeroSpeedCommand())
      .andThen(shooterArm.shooterArmScoreLOWCommand());

      // Combine the LED command with the main command sequence
      Command command = ledCommand.andThen(mainCommand);

      command.addRequirements(shooter, shooterArm, elevator, funnel, algaeArm, algaeShooter);

      return command;
  }
  
  

  public static Command setIntakeCommandFORAUTOONLY(Shooter shooter, ShooterArm shooterArm, Elevator elevator, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    
    
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
      .andThen(shooterArm.shooterArmLoadCommand())
      .andThen(shooter.shooterIntakeCommand())
      .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
      .andThen(shooter.shooterZeroSpeedCommand())
      .andThen(shooterArm.shooterArmScoreLOWCommand());
  
  
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
        .andThen(new WaitCommand(1))
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

  public static Command setClimbPositionNoArm(AlgaeArm algaeArm, Funnel funnel, Climber climber) {

    Command command = algaeArm.algaeArmStraightOutCommand()
    .andThen(funnel.funnelFullUpCommand())
    // Add climber control - this will enable position mode temporarily
    .andThen(climber.climberFullExtendCommand());

    command.addRequirements(algaeArm, funnel, climber);

    return command;
  }

  public static Command setClimbPositionArmOnly(Elevator elevator, ShooterArm shooterArm) {
    Command command = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorClimbPoseCommand());

    command.addRequirements(shooterArm, elevator);

    return command;
  }

  public static Command pullOffHighAboveBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreBallCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  public static Command pullOffHighBelowBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallBelowCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreBallBelowCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }
  
  public static Command pullOffLowBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreLowBallCommand());


    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  public static Command ballDown(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmBallCommand();

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }
  
  public static Command scoreL1CommandNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, AlgaeArm algaeArm, AlgaeShooter algaeShooter, Funnel funnel) {
      
    Command command  = elevator.setElevatorL1Command()
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmOutLoadCommand())
    .andThen(algaeArm.algaeArmScoreL1Command())
    .andThen(algaeShooter.algaeShooterIntakeCommand())
    .andThen(funnel.funnelL1Command());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

public static Command scoreBasedOnQueueCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmBasedOnQueueCommand(buttonBox)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevateBasedOnQueue(buttonBox)))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .alongWith(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator, drivebase);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRST(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
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

public static Command sourceDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, Funnel funnel){

  Command command = drivebase.startDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(1.5))
  .andThen(CommandFactory.setIntakeCommandFORAUTOONLY(shooter, shooterArm, elevator, drivebase, robotContainer));

    command.addRequirements(shooter, shooterArm, elevator, funnel);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C530"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}

public static Command RightAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C330"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C230")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C231")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(shooter, shooterArm, elevator, funnel);
    return command; 
}

public static Command LeftCenterAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C431"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand());
  
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}
public static Command RightCenterAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C430"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand());
  
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}
}