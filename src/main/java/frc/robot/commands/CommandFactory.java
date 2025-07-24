package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Algae.AlgaeShooter;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Climber;

import frc.robot.util.DynamicWait;

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
      .andThen(shooter.shooterZeroSpeedCommand())
      .andThen(shooterArm.shooterArmScoreLOWCommand())
      .andThen(new InstantCommand(() -> shooter.setisL1ScoringFalse()));

      // Combine the LED command with the main command sequence
      Command command = ledCommand.andThen(mainCommand);

      command.addRequirements(shooter, shooterArm, elevator, funnel, algaeArm, algaeShooter);

      return command;
  }
  
  

  public static Command setIntakeCommandFORAUTOONLY(Shooter shooter, ShooterArm shooterArm, Elevator elevator, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter) {
    
    
      Command command  = elevator.setElevatorPickupCommand()
      .andThen(funnel.funnelHomeCommand())
      .andThen(algaeArm.algaeArmStowUpCommand())
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
    Command command = elevator.setElevatorPickupCommand()
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmLoadCommand());

    command.addRequirements(shooterArm, elevator);

    return command;
  }

  public static Command pullOffHighAboveBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreBallCommand())
    .andThen(shooter.shooterOutakeCommand());

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  public static Command pullOffLowBall(Shooter shooter, ShooterArm shooterArm, Elevator elevator) {
      
    Command command  = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreLowBallCommand())
    .andThen(shooter.shooterOutakeCommand());



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
    .andThen(shooterArm.shooterArmScoreL1RealCommand())
    .andThen(algaeArm.algaeArmScoreL1Command())
    .andThen(funnel.funnelL1Command())
    .andThen(new InstantCommand(() -> shooter.setisL1ScoringTrue()));

    command.addRequirements(shooter, shooterArm, elevator);

    return command;
  }

  public static Command finishL1ScoreCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, AlgaeArm algaeArm, AlgaeShooter algaeShooter, Funnel funnel) {

    Command command = new WaitCommand(0.3)
    .andThen(funnel.funnelL1DumpCommand())
    .andThen(elevator.setElevatorL1Command())
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmLoadCommand())
    .andThen(new InstantCommand(() -> shooter.setisL1ScoringFalse()));

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
  
public static Command scoreBasedOnQueueCommandRight(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmBasedOnQueueCommandRight(buttonBox)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevateBasedOnQueueRight(buttonBox)))
    .andThen(elevator.elevatorBasedOnQueueCommandRight(buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandLeft(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox){

  Command command = shooterArm.shooterArmBasedOnQueueCommandLeft(buttonBox)
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevateBasedOnQueueLeft(buttonBox)))
    .andThen(elevator.elevatorBasedOnQueueCommandLeft(buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

// New combined command that scores right target and drives to next target ending in "1"
public static Command scoreBasedOnQueueCommandRightWithDrive(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase) {
  return scoreBasedOnQueueCommandRight(shooter, shooterArm, elevator, buttonBox)
    .andThen(new InstantCommand(() -> {
      // Create a custom ButtonBox wrapper that filters by suffix "1"
      ButtonBox filteredButtonBox = new ButtonBox(drivebase) {
        @Override
        public TargetClass peekNextTarget() {
          return buttonBox.peekNextTargetEndingIn1();
        }
      };
      // Start the drive command in the background
      drivebase.startDriveToPose(filteredButtonBox, elevator).schedule();
    }));
}

// New combined command that scores left target and drives to next target ending in "0"  
public static Command scoreBasedOnQueueCommandLeftWithDrive(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase) {
  return scoreBasedOnQueueCommandLeft(shooter, shooterArm, elevator, buttonBox)
    .andThen(new InstantCommand(() -> {
      // Create a custom ButtonBox wrapper that filters by suffix "0"
      ButtonBox filteredButtonBox = new ButtonBox(drivebase) {
        @Override
        public TargetClass peekNextTarget() {
          return buttonBox.peekNextTargetEndingIn0();
        }
      };
      // Start the drive command in the background
      drivebase.startDriveToPose(filteredButtonBox, elevator).schedule();
    }));
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .alongWith(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator, drivebase);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRST(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(shooter.shooterOutakeCommand())
  .andThen(new WaitCommand(.25))
  .andThen(shooter.shooterZeroSpeedCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startSlowDriveToPose(buttonBox, elevator)
  .andThen(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(shooter.shooterOutakeCommand())
  .andThen(new WaitCommand(.25))
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
  .andThen(new WaitCommand(.25))
  .andThen(shooter.shooterZeroSpeedCommand());
    
    command.addRequirements(shooter, shooterArm, elevator);
    return command; 
}

public static Command sourceDriveAuto(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter) {

  // Use startFastDriveToPoseWithRotationDelay instead of the regular one for faster source driving
  Command command = drivebase.startFastDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(.75))
  //.andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  //.andThen(new WaitCommand(1.25));
  .andThen(CommandFactory.setIntakeCommandFORAUTOONLY(shooter, shooterArm, elevator, drivebase, robotContainer, funnel, algaeArm, algaeShooter));

  command.addRequirements(shooter, shooterArm, elevator, funnel);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S530"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C631")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}

public static Command RightAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter){

    Command command = new InstantCommand(() -> buttonBox.addTarget("S331"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C230")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C231")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(shooterArm.shooterArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(shooter, shooterArm, elevator, buttonBox, robotContainer, drivebase, funnel, algaeArm, algaeShooter))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(shooter, shooterArm, elevator, funnel);
    return command; 
}

public static Command LeftCenterAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S431"))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A511")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A510")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A500")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A411")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A410")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A400")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A311")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A310")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A300")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> DynamicWait.resetAutoCounter()));
  
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}
public static Command RightCenterAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S430"))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(shooterArm.shooterArmScoreLOWCommand())
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A311")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A310")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A300")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A411")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A410")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A400")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A511")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A510")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A500")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> DynamicWait.resetAutoCounter()));
  
  command.addRequirements(shooter, shooterArm, elevator, funnel);
  return command; 
}

public static Command algaeRemoveBasedOnQueueCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    Command command = shooterArm.shooterArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))
    .andThen(buttonBox.setElevatorForCurrentBallCommand(elevator))
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(shooterArm.shooterArmPreLowBallCommand())
    .andThen(shooter.shooterOutakeCommand());

    command.addRequirements(shooter, shooterArm, elevator);
    return command;
}

public static Command algaeRemoveBasedOnQueueCommandDriveCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    Command command = drivebase.startDriveToPose(buttonBox, elevator)
    .andThen(CommandFactory.algaeRemoveBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitUntilCommand(robotContainer.veryCloseTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startDriveToPose(buttonBox, elevator));


    command.addRequirements(shooter, shooterArm, elevator);
    return command;
}

public static Command algaeRemoveBasedOnQueueCommandDriveAutoCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {

    Command command = drivebase.startDriveToPosePATHPLANNER(buttonBox, elevator)
    .andThen(CommandFactory.algaeRemoveBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitUntilCommand(robotContainer.closeTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startFastDriveToPose(buttonBox, elevator))
    .andThen(new WaitUntilCommand(robotContainer.veryCloseTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startDriveToPose(buttonBox, elevator))
    .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()));

    command.addRequirements(shooter, shooterArm, elevator);
    return command;
}
}