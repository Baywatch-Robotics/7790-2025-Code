package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetClass;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import frc.robot.util.DynamicWait;

public class CommandFactory {

   
    public static Command setCoralIntakeCommand(EndEffector endEffector, Arm arm, Elevator elevator, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer) {
      
      // Run the LED pattern first as a separate command
      Command ledCommand = led.runPattern("INTAKE_PATTERN");
      
      Command intakeStart = intake.deployCommand()
        .andThen(intake.intakeCommand())
        .andThen(indexer.indexCommand())
        .andThen(elevator.setElevatorHoverCommand())
        .andThen(arm.ArmPickUpCommand())
        .andThen(endEffector.endEffectorIntakeCommand())
        .andThen(new WaitUntilCommand(indexer.coralIndexedTrigger()))
        .andThen(indexer.stopCommand())
        .andThen(intake.stopCommand())
        .andThen(elevator.setElevatorPickupCommand())
        .andThen(new WaitUntilCommand(endEffector.coralLoadedTrigger()))
        .andThen(endEffector.endEffectorZeroSpeedCommand())
        .andThen(elevator.setElevatorHoverCommand())
        .andThen(intake.stowCommand());

      Command command = ledCommand
        .andThen(intakeStart);

      command.addRequirements(endEffector, arm, elevator, intake);

      return command;
  }
  
  

  public static Command setLollipopIntakeCommand(EndEffector endEffector, Arm arm, Elevator elevator, RobotContainer robotContainer, LED led) {
    
    
      Command command  = led.runPattern("INTAKE_PATTERN")
      .andThen (endEffector.endEffectorIntakeCommand())
      .andThen(arm.ArmLollipopCommand())
      .andThen(new WaitUntilCommand(arm.isClearToDescend()))
      .andThen(elevator.setElevatorLollipopCommand())
      .andThen(new WaitUntilCommand(endEffector.coralLoadedTrigger()))
      .andThen(endEffector.endEffectorZeroSpeedCommand());
  
  
      command.addRequirements(endEffector, arm, elevator);
  
      return command;
  }

  public static Command setElevatorZero(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = arm.ArmScoreLOWCommand() //Will make this straight up at some point
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
      .andThen(elevator.setfullElevatorRetractCommand());


    command.addRequirements(endEffector, arm, elevator);

    return command;
  }

  public static Command intakeHighBall(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = arm.ArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(endEffector.endEffectorIntakeCommand());

    command.addRequirements(endEffector, arm, elevator);

    return command;
  }

  public static Command intakeLowBall(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = arm.ArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(endEffector.endEffectorIntakeCommand());



    command.addRequirements(endEffector, arm, elevator);

    return command;
  }
  
  public static Command scoreL1CommandNOSHOOT(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = elevator.setElevatorL1Command()
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(arm.ArmScoreL1RealCommand())
    .andThen(new InstantCommand(() -> endEffector.setisL1ScoringTrue()));

    command.addRequirements(endEffector, arm, elevator);

    return command;
  }
  
public static Command scoreBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommand(buttonBox)
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandRight(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommandRight(buttonBox)
    .andThen(elevator.elevatorBasedOnQueueCommandRight(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandLeft(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommandLeft(buttonBox)
    .andThen(elevator.elevatorBasedOnQueueCommandLeft(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

// New combined command that scores right target and drives to next target ending in "1"
public static Command scoreBasedOnQueueCommandRightWithDrive(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase) {
  return scoreBasedOnQueueCommandRight(endEffector, arm, elevator, buttonBox)
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
public static Command scoreBasedOnQueueCommandLeftWithDrive(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase) {
  return scoreBasedOnQueueCommandLeft(endEffector, arm, elevator, buttonBox)
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

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .alongWith(CommandFactory.scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox));
    
    command.addRequirements(endEffector, arm, elevator, drivebase);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRST(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(CommandFactory.scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(endEffector.endEffectorOuttakeCommand())
  .andThen(new WaitCommand(.25))
  .andThen(endEffector.endEffectorZeroSpeedCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startSlowDriveToPose(buttonBox, elevator)
  .andThen(CommandFactory.scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(endEffector.endEffectorOuttakeCommand())
  .andThen(new WaitCommand(.25))
  .andThen(endEffector.endEffectorZeroSpeedCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAuto(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .andThen(new WaitUntilCommand(robotContainer.approachingTrigger()))
  .andThen(CommandFactory.scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(endEffector.endEffectorOuttakeCommand())
  .andThen(new WaitCommand(.25))
  .andThen(endEffector.endEffectorZeroSpeedCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command sourceDriveAuto(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, LED led, Intake intake, Indexer indexer){ {

  // Use startFastDriveToPoseWithRotationDelay instead of the regular one for faster source driving
  Command command = drivebase.startFastDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(.75))
  //.andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  //.andThen(new WaitCommand(1.25));
  .andThen(CommandFactory.setCoralIntakeCommand(endEffector, arm, elevator, robotContainer, led, intake, indexer));

  command.addRequirements(endEffector, arm, elevator);

  return command;} 
}

public static Command lollipopIntakeDriveAuto(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, LED led, Intake intake, Indexer indexer){ {

  // Use startFastDriveToPoseWithRotationDelay instead of the regular one for faster source driving
  Command command = drivebase.startDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(.25))
  .andThen(CommandFactory.setCoralIntakeCommand(endEffector, arm, elevator, robotContainer, led, intake, indexer));

  command.addRequirements(endEffector, arm, elevator);

  return command;} 
}

public static Command lollipopDriveAuto(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, LED led, Intake intake, Indexer indexer){ {

  // Use startFastDriveToPoseWithRotationDelay instead of the regular one for faster source driving
  Command command = drivebase.startDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(.25))
  .andThen(CommandFactory.setLollipopIntakeCommand(endEffector, arm, elevator, robotContainer, led));

  command.addRequirements(endEffector, arm, elevator);

  return command;} 
}

public static Command LeftIntakeLollipopAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

  double firstDriveWaitSeconds = 2.5;
  Command command = new InstantCommand(() -> buttonBox.addTarget("PL0"))
  .andThen(new InstantCommand(() -> { drivebase.startDriveToPose(buttonBox, elevator).schedule();}))
  .andThen(new WaitCommand(firstDriveWaitSeconds))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1300")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LLI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1100")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCLI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1310")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LRI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1110")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command RightIntakeLollipopAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

  double firstDriveWaitSeconds = 2.5;
  Command command = new InstantCommand(() -> buttonBox.addTarget("PR0"))
  .andThen(new InstantCommand(() -> { drivebase.startDriveToPose(buttonBox, elevator).schedule();}))
  .andThen(new WaitCommand(firstDriveWaitSeconds))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1310")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LRI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1110")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCRI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1300")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LLI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1100")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCI")))
  .andThen(CommandFactory.lollipopIntakeDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command LeftLollipopAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

  double firstDriveWaitSeconds = 2.5;
  Command command = new InstantCommand(() -> buttonBox.addTarget("PL1"))
  .andThen(new InstantCommand(() -> { drivebase.startDriveToPose(buttonBox, elevator).schedule();}))
  .andThen(new WaitCommand(firstDriveWaitSeconds))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1301")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LL")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1101")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCL")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1311")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LR")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1111")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LC")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command RightLollipopAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

  double firstDriveWaitSeconds = 2.5;
  Command command = new InstantCommand(() -> buttonBox.addTarget("PR1"))
  .andThen(new InstantCommand(() -> { drivebase.startDriveToPose(buttonBox, elevator).schedule();}))
  .andThen(new WaitCommand(firstDriveWaitSeconds))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1311")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LR")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1111")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LCR")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1301")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LL")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1101")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("LC")))
  .andThen(CommandFactory.lollipopDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command LeftAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S530"))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(arm.ArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C630")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(arm.ArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C631")))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(arm.ArmScoreLOWCommand())
  .andThen(elevator.setElevatorPickupCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command RightAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("S331"))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(arm.ArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C230")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(arm.ArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C231")))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    .andThen(arm.ArmScoreLOWCommand())
    .andThen(elevator.setElevatorPickupCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command LeftCenterAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S431"))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(arm.ArmScoreLOWCommand())
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A511")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A510")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A500")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A411")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A410")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A400")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A311")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A310")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A300")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> DynamicWait.resetAutoCounter()));
  
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}
public static Command RightCenterAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = new InstantCommand(() -> buttonBox.addTarget("S430"))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(arm.ArmScoreLOWCommand())
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A311")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A310")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A300")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A411")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A410")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A400")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A511")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A510")))
  .andThen(new InstantCommand(() -> buttonBox.addTarget("A500")))
  .andThen(DynamicWait.dynamicIncrementWaitCommand())
  .andThen(algaeRemoveBasedOnQueueCommandDriveAutoCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  .andThen(new InstantCommand(() -> DynamicWait.resetAutoCounter()));
  
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command algaeRemoveBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    Command command = arm.ArmScoreLOWCommand()
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
    .andThen(buttonBox.setElevatorForCurrentBallCommand(elevator))
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(endEffector.endEffectorOuttakeCommand());

    command.addRequirements(endEffector, arm, elevator);
    return command;
}

public static Command algaeRemoveBasedOnQueueCommandDriveCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    Command command = drivebase.startDriveToPose(buttonBox, elevator)
    .andThen(CommandFactory.algaeRemoveBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitUntilCommand(robotContainer.veryCloseTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startDriveToPose(buttonBox, elevator));


    command.addRequirements(endEffector, arm, elevator);
    return command;
}

public static Command algaeRemoveBasedOnQueueCommandDriveAutoCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {

    Command command = drivebase.startDriveToPosePATHPLANNER(buttonBox, elevator)
    .andThen(CommandFactory.algaeRemoveBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitUntilCommand(robotContainer.closeTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startFastDriveToPose(buttonBox, elevator))
    .andThen(new WaitUntilCommand(robotContainer.veryCloseTrigger()))
    .andThen(buttonBox.getNextTargetCommand())
    .andThen(drivebase.startDriveToPose(buttonBox, elevator))
    .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()));

    command.addRequirements(endEffector, arm, elevator);
    return command;
}

public static Command algaeIntakeBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox) {
    Command command = arm.ArmBasedOnQueueCommand(buttonBox)
        .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))
        .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
        .andThen(endEffector.endEffectorIntakeCommand());
        
    command.addRequirements(endEffector, arm, elevator);
    return command;
}
}