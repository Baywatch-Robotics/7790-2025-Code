package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae.AlgaeArm;
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
      .andThen(elevator.setfullElevatorRetractCommand());


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
    // Add vision fine-tuning if needed after mechanisms are positioned
    .andThen(() -> {
      // Get current level from button box
      int level = buttonBox.currentLevelSupplier.getAsInt();
      // Fine-tune elevator height using vision if scope is available
      if (level >= 2 && elevator.getScope() != null && 
          elevator.getScope().isVisionEnabled() && 
          elevator.getScope().hasTarget()) {
        elevator.autoTargetForLevelCommand(level, elevator.getScope()).schedule();
      }
      // Fine-tune pivot angle using vision if scope is available
      if (shooterPivot.scope != null && 
          shooterPivot.scope.isVisionEnabled() && 
          shooterPivot.scope.hasTarget()) {
        shooterPivot.moveAmount((float)shooterPivot.scope.calculatePivotAngleAdjustment());
      }
    })
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(shooter.shooterIntakeCommand())

    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))

    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))

    .andThen(shooter.shooterZeroSpeedCommand())


    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))

    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))

    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))

    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
    .andThen(drivebase.driveToPose(buttonBox))
    .andThen(new WaitUntilCommand(robotContainer.isVeryCloseTrigger())) // Wait until very close to target
    // Enable continuous aiming when very close to target
    .andThen(() -> {
      int level = buttonBox.currentLevelSupplier.getAsInt();
      if (shooterPivot.scope != null) {
        shooterPivot.scope.enableContinuousAiming(level);
        SmartDashboard.putString("Aiming Status", "Continuous Aiming Started");
      }
    })
    // Wait for aiming to stabilize or timeout after 1 second
    .andThen(new WaitUntilCommand(() -> 
      (shooterPivot.scope != null && shooterPivot.scope.isAimingStable()) || 
      !shooterPivot.scope.hasTarget()
    ).withTimeout(1.0))
    // Disable continuous aiming before completing
    .andThen(() -> {
      if (shooterPivot.scope != null) {
        shooterPivot.scope.disableContinuousAiming();
        SmartDashboard.putString("Aiming Status", "Aiming Complete");
      }
    })
    .andThen(buttonBox.getNextTargetCommand());
    
    command.addRequirements(algaeArm, shooter, shooterArm, shooterPivot, elevator);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAuto(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator)
    .andThen(shooter.shooterIntakeCommand())

    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))

    .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))

    .andThen(shooter.shooterZeroSpeedCommand())


    .andThen(new WaitUntilCommand(shooterArm.isClearToElevate()))

    .andThen(elevator.elevatorBasedOnQueueCommand(buttonBox))

    .andThen(new WaitUntilCommand(elevator.isAtSetpoint()))

    .andThen(shooterPivot.shooterPivotBasedOnQueueCommand(buttonBox))
    .andThen(drivebase.driveToPose(buttonBox))
    .andThen(new WaitUntilCommand(robotContainer.isLinedUpTrigger()))
    // Add vision adjustments before shooting
    .andThen(() -> {
      // Apply vision adjustments if available
      if (shooterPivot.scope != null && 
          shooterPivot.scope.isVisionEnabled() && 
          shooterPivot.scope.hasTarget()) {
        // Apply fine adjustments for pivot
        double adjustment = shooterPivot.scope.calculatePivotAngleAdjustment();
        shooterPivot.moveAmount((float)adjustment);
        SmartDashboard.putNumber("Final Pivot Adjustment", adjustment);
      }
      
      // Apply vision adjustments to elevator if available
      if (elevator.getScope() != null && 
          elevator.getScope().isVisionEnabled() && 
          elevator.getScope().hasTarget()) {
        int level = buttonBox.currentLevelSupplier.getAsInt();
        double[] heightAdjustments = elevator.getScope().calculateHeightForLevel(level);
        if (heightAdjustments.length > 0) {
          elevator.adjustForVision(heightAdjustments[0]);
          SmartDashboard.putNumber("Final Elevator Adjustment", heightAdjustments[0]);
        }
      }
    })
    .andThen(new WaitCommand(0.5)) // Small pause to let adjustments settle
    .andThen(shooter.shooterOutakeCommand())
    .andThen(new WaitCommand(.5))
    .andThen(shooter.shooterZeroSpeedCommand())
    // Disable continuous aiming after shooting
    .andThen(() -> {
      if (shooterPivot.scope != null) {
        shooterPivot.scope.disableContinuousAiming();
        SmartDashboard.putString("Aiming Status", "Aiming Complete");
      }
    })
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
    .andThen(new WaitUntilCommand(robotContainer.isVeryCloseTrigger()))
    .andThen(elevator.setElevatorPickupCommand())
    .andThen(new WaitUntilCommand(elevator.isClearToIntake()))
    .andThen(shooterArm.shooterArmLoadCommand())
    .alongWith(shooterPivot.setCenterCommand())
    .andThen(shooter.shooterIntakeCommand())
    .andThen(new WaitUntilCommand(shooter.coralLoadedTrigger()))
    .andThen(elevator.setElevatorPickupPlusCommand())
    .andThen(new WaitCommand(1))
    .andThen(shooter.shooterZeroSpeedCommand())
    .andThen(CommandFactory.setElevatorZero(algaeArm, shooter, shooterArm, shooterPivot, elevator));

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

  Command command = drivebase.pathfindThenFollowPath("Left Auton Start")
  .andThen(new InstantCommand(() -> buttonBox.addTarget("C530")))
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
  
  .andThen(drivebase.pathfindThenFollowPath("Left to 6"))
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
  .andThen(drivebase.pathfindThenFollowPath("Left to 6"))
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

    Command command = drivebase.pathfindThenFollowPath("Right Auton Start")
    .andThen(new InstantCommand(() -> buttonBox.addTarget("C330")))
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
    
    .andThen(drivebase.pathfindThenFollowPath("Right to 2"))
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

    // Score at position 2 again
    .andThen(drivebase.pathfindThenFollowPath("Right to 2"))
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