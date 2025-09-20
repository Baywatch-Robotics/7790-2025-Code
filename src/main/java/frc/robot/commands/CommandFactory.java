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

public class CommandFactory {

   
    public static Command setCoralIntakeCommand(EndEffector endEffector, Arm arm, Elevator elevator, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer) {
      
      Command ledCommand = led.runPattern("INTAKE_PATTERN");
      
      Command intakeStart = intake.deployCommand()
        .andThen(intake.intakeCommand())
        .andThen(indexer.indexCommand())
        .andThen(elevator.setElevatorHoverCommand())
        .andThen(new WaitCommand(0.25))
        .andThen(arm.ArmPickUpCommand())
        ;

      Command command = ledCommand
        .andThen(intakeStart);

      command.addRequirements(endEffector, arm, elevator, intake);

      return command;
  }
  public static Command setCoralFinishIntakeCommand(EndEffector endEffector, Arm arm, Elevator elevator, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer) {
    
    Command ledCommand = led.runPattern("INTAKE_PATTERN");
    
    Command intakeStart = indexer.stopCommand()
      .andThen(intake.stopCommand())
      .andThen(endEffector.endEffectorIntakeCommand())
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

  public static Command setCoralOuttakeCommand(Intake intake, Indexer indexer, RobotContainer robotContainer, LED led) {
    
    
      Command command  = led.runPattern("INTAKE_PATTERN")
      .andThen(intake.deployCommand())
      .andThen(intake.outtakeCommand())
      .andThen(indexer.reverseCommand());
  
  
      command.addRequirements(intake, indexer);
  
      return command;
  }
  
  

  public static Command setLollipopIntakeCommand(EndEffector endEffector, Arm arm, Elevator elevator, RobotContainer robotContainer, LED led) {
    
    
      Command command  = led.runPattern("INTAKE_PATTERN")
      .andThen(endEffector.endEffectorIntakeCommand())
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
      
    Command command  = arm.ArmReefAlgaeCommand()
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
    .andThen(elevator.setElevatorHighBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(endEffector.endEffectorIntakeCommand());

    command.addRequirements(endEffector, arm, elevator);

    return command;
  }

  public static Command intakeLowBall(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = arm.ArmReefAlgaeCommand()
    .andThen(new WaitUntilCommand(arm.isClearToElevate()))
    .andThen(elevator.setElevatorLowBallCommand())
    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))
    .andThen(endEffector.endEffectorIntakeCommand());



    command.addRequirements(endEffector, arm, elevator);

    return command;
  }
  
  public static Command scoreL1CommandNOSHOOT(EndEffector endEffector, Arm arm, Elevator elevator) {
      
    Command command  = elevator.setElevatorL1Command()
    .andThen(arm.ArmScoreL1Command());

    command.addRequirements(endEffector, arm, elevator);

    return command;
  }
  
public static Command scoreBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommand(buttonBox)
    .andThen(new WaitCommand(.5))
    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandFirst(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = elevator.elevatorBasedOnQueueCommand(buttonBox)
    .andThen(new WaitCommand(1))
    .andThen(arm.ArmBasedOnQueueCommand(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandRight(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommandRight(buttonBox)
    .andThen(new WaitCommand(.5))
    .andThen(elevator.elevatorBasedOnQueueCommandRight(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandLeft(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = arm.ArmBasedOnQueueCommandLeft(buttonBox)
    .andThen(new WaitCommand(.5))
    .andThen(elevator.elevatorBasedOnQueueCommandLeft(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandRightFirst(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = elevator.elevatorBasedOnQueueCommandRight(buttonBox)
    .andThen(new WaitCommand(3))
    .andThen(arm.ArmBasedOnQueueCommandRight(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command scoreBasedOnQueueCommandLeftFirst(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox){

  Command command = elevator.elevatorBasedOnQueueCommandLeft(buttonBox)
    .andThen(new WaitCommand(3))
    .andThen(arm.ArmBasedOnQueueCommandLeft(buttonBox));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}
  
public static Command placeBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase){
  Command command =
    arm.ArmPlaceBasedOnQueueCommand(buttonBox)
    .andThen(new WaitCommand(.5))
    .andThen(elevator.elevatorPlaceBasedOnQueueCommand(buttonBox))
    .andThen(new WaitCommand(.25))
    .andThen(endEffector.endEffectorOuttakeCommand())
    // Post-step: after placing, if queue head is Coral level 3, enqueue "C{face}3{xx}P" and start driving
    .andThen(new InstantCommand(() -> {
      TargetClass t = buttonBox.peekNextTarget();
      if (t == null) return;
      String name = t.getName();
      if (name == null) return;
      // Require: starts with 'C', has at least 5 chars for C F L X X, and is level 3
      if (name.startsWith("A") || name.length() < 5 || name.charAt(2) != '3') return;

      char faceChar = name.charAt(1);
      String xx = name.substring(3, 5); // "00", "01", "10", "11"
      String newName;
      switch (xx) {
        case "00": newName = "C" + faceChar + "3" + "00" + "P"; break;
        case "01": newName = "C" + faceChar + "3" + "01" + "P"; break;
        case "10": newName = "C" + faceChar + "3" + "10" + "P"; break;
        case "11": newName = "C" + faceChar + "3" + "11" + "P"; break;
        default: return; // Unknown pattern, do nothing
      }
      // Clear existing queue, then enqueue the new "C?3??P" and start driving
      buttonBox.clearTargets();
      buttonBox.addTarget(newName);
      drivebase.startDriveToPose(buttonBox, elevator).schedule();
    }))
    .andThen(new WaitCommand(1.0))
    .andThen(drivebase.stopDriveToPoseCommand())
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

// New combined command that scores right target and drives to next target ending in "1"
public static Command scoreBasedOnQueueCommandRightWithDrive(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase) {
  return scoreBasedOnQueueCommandRight(endEffector, arm, elevator, buttonBox)
  .andThen(new WaitCommand(.5))
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
  .andThen(new WaitCommand(.5))
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

  Command command = CommandFactory.scoreBasedOnQueueCommandFirst(endEffector, arm, elevator, buttonBox)
  .andThen(new WaitCommand(1))
  .andThen(drivebase.startDriveToPose(buttonBox, elevator))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(CommandFactory.placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(new WaitCommand(.25))
  .andThen(endEffector.endEffectorZeroSpeedCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command;
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.scoreBasedOnQueueCommandFirst(endEffector, arm, elevator, buttonBox)
  .andThen(new WaitCommand(2))
  .andThen(drivebase.startSlowDriveToPose(buttonBox, elevator))
  .andThen(new WaitUntilCommand(robotContainer.linedUpTrigger()))
  .andThen(CommandFactory.placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
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
  .andThen(CommandFactory.placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(new WaitCommand(.25))
  .andThen(endEffector.endEffectorZeroSpeedCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command sourceDriveAuto(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, RobotContainer robotContainer, SwerveSubsystem drivebase, LED led, Intake intake, Indexer indexer){ {

  // Use startFastDriveToPoseWithRotationDelay instead of the regular one for faster source driving
  Command command = drivebase.startFastDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(new WaitCommand(0.75))
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

  Command command = new InstantCommand(() -> buttonBox.addTarget("S5300"))
  .andThen(intake.stowCommand())
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(arm.ArmPickUpCommand())
  .andThen(elevator.setElevatorHoverCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("PSL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C6300")))
  .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitCommand(.5))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(arm.ArmPickUpCommand())
  .andThen(elevator.setElevatorHoverCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("PSL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
  
  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C6310")))
  .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitCommand(.5))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(arm.ArmPickUpCommand())
  .andThen(elevator.setElevatorHoverCommand())

  .andThen(new InstantCommand(() -> buttonBox.addTarget("PSL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("SL")))
  .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
  .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

  .andThen(new InstantCommand(() -> buttonBox.addTarget("C1300")))
  .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
  .andThen(new WaitCommand(.5))
  .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
  .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
  .andThen(arm.ArmPickUpCommand())
  .andThen(elevator.setElevatorHoverCommand());
    
  command.addRequirements(endEffector, arm, elevator);
  return command; 
}

public static Command RightAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("S3310"))
    .andThen(intake.stowCommand())
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRST(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmPickUpCommand())
    .andThen(elevator.setElevatorHoverCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("PSR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C2300")))
    .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
    .andThen(new WaitCommand(.5))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmPickUpCommand())
    .andThen(elevator.setElevatorHoverCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("PSR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C2310")))
    .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
    .andThen(new WaitCommand(.5))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmPickUpCommand())
    .andThen(elevator.setElevatorHoverCommand())

    .andThen(new InstantCommand(() -> buttonBox.addTarget("PSR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))

    .andThen(new InstantCommand(() -> buttonBox.addTarget("SR")))
    .andThen(CommandFactory.sourceDriveAuto(endEffector, arm, elevator, buttonBox, robotContainer, drivebase, led, intake, indexer))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()))
    
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C1310")))
    .andThen(scoreBasedOnQueueCommand(endEffector, arm, elevator, buttonBox))
    .andThen(new WaitCommand(.5))
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAuto(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmPickUpCommand())
    .andThen(elevator.setElevatorHoverCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command RightCenterAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C4310"))
    .andThen(intake.stowCommand())
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitCommand(1))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmScoreHIGHCommand())
    .andThen(elevator.setElevatorHoverCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command LeftCenterAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("C4300"))
    .andThen(intake.stowCommand())
    .andThen(CommandFactory.scoreBasedOnQueueCommandDriveAutoFIRSTBACKAUTO(endEffector, arm, elevator, buttonBox, drivebase, robotContainer))
    .andThen(new WaitCommand(1))
    .andThen(placeBasedOnQueueCommand(endEffector, arm, elevator, buttonBox, drivebase))
    .andThen(arm.ArmScoreHIGHCommand())
    .andThen(elevator.setElevatorHoverCommand());
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command DriveAutonCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, LED led, Intake intake, Indexer indexer){

    Command command = new InstantCommand(() -> buttonBox.addTarget("D"))
    .andThen(intake.stowCommand())
    .andThen(elevator.setElevatorL2Command())
    .andThen(new InstantCommand(() -> { drivebase.startDriveToPose(buttonBox, elevator).schedule();}))
    .andThen(new WaitCommand(5))
    .andThen(new InstantCommand(() -> buttonBox.clearTargets()));
    
    command.addRequirements(endEffector, arm, elevator);
    return command; 
}

public static Command algaeRemoveBasedOnQueueCommand(EndEffector endEffector, Arm arm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer) {
    Command command = arm.ArmReefAlgaeCommand()
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