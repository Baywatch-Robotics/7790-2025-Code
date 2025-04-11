package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoAlgaeReefRemovalConstants;
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

public static Command scoreBasedOnQueueCommandDriveAutoNOSHOOT(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPose(buttonBox, elevator)
  .alongWith(CommandFactory.scoreBasedOnQueueCommand(shooter, shooterArm, elevator, buttonBox));
    
    command.addRequirements(shooter, shooterArm, elevator, drivebase);
    return command; 
}

public static Command scoreBasedOnQueueCommandDriveAutoFIRST(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer){

  Command command = drivebase.startDriveToPoseWithRotationDelay(buttonBox, elevator)
  .andThen(shooterArm.shooterArmBasedOnQueueCommand(buttonBox))
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
  .andThen(CommandFactory.setIntakeCommandFORAUTOONLY(shooter, shooterArm, elevator, drivebase, robotContainer, funnel, algaeArm, algaeShooter));

  command.addRequirements(shooter, shooterArm, elevator, funnel);

  return command; 
}

public static Command LeftAutonCommand(Shooter shooter, ShooterArm shooterArm, Elevator elevator, ButtonBox buttonBox, SwerveSubsystem drivebase, RobotContainer robotContainer, Funnel funnel, AlgaeArm algaeArm, AlgaeShooter algaeShooter){

  Command command = new InstantCommand(() -> buttonBox.addTarget("C530"))
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

    Command command = new InstantCommand(() -> buttonBox.addTarget("C331"))
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

   /**
    * Creates a command sequence for automatic algae reef removal
    * 
    * @param shooter The shooter subsystem
    * @param shooterArm The shooter arm subsystem
    * @param elevator The elevator subsystem
    * @param drivebase The swerve drive subsystem
    * @param faceNumber The reef face number to target (1-6)
    * @param isHighBall Whether to use high ball (true) or low ball (false) retrieval
    * @param isLeftSide Whether to use left side (true) or right side (false) of target
    * @return The command sequence
    */
   public static Command autoAlgaeReefRemovalCommand(
         Shooter shooter, 
         ShooterArm shooterArm, 
         Elevator elevator,
         SwerveSubsystem drivebase,
         ButtonBox buttonBox,
         int faceNumber,
         boolean isHighBall,
         boolean isLeftSide) {
       
      // Initialize SmartDashboard values if they don't exist
      if (!SmartDashboard.containsKey(AutoAlgaeReefRemovalConstants.BACKUP_DRIVE_WAIT_KEY)) {
         SmartDashboard.putNumber(AutoAlgaeReefRemovalConstants.BACKUP_DRIVE_WAIT_KEY, 
               AutoAlgaeReefRemovalConstants.DEFAULT_BACKUP_WAIT_TIME);
      }
      if (!SmartDashboard.containsKey(AutoAlgaeReefRemovalConstants.ARM_CONFIG_WAIT_KEY)) {
         SmartDashboard.putNumber(AutoAlgaeReefRemovalConstants.ARM_CONFIG_WAIT_KEY, 
               AutoAlgaeReefRemovalConstants.DEFAULT_ARM_CONFIG_WAIT_TIME);
      }
      if (!SmartDashboard.containsKey(AutoAlgaeReefRemovalConstants.CROSS_DRIVE_WAIT_KEY)) {
         SmartDashboard.putNumber(AutoAlgaeReefRemovalConstants.CROSS_DRIVE_WAIT_KEY, 
               AutoAlgaeReefRemovalConstants.DEFAULT_CROSS_DRIVE_WAIT_TIME);
      }
      if (!SmartDashboard.containsKey(AutoAlgaeReefRemovalConstants.PULLBACK_WAIT_KEY)) {
         SmartDashboard.putNumber(AutoAlgaeReefRemovalConstants.PULLBACK_WAIT_KEY, 
               AutoAlgaeReefRemovalConstants.DEFAULT_PULLBACK_WAIT_TIME);
      }
      
      // Determine target designation based on face number and side
      String backupTarget = String.format("A%dX%dBackup", faceNumber, isLeftSide ? 0 : 1);
      String finalTarget = String.format("A%dX%d", faceNumber, isLeftSide ? 0 : 1);
      
      // Build the command sequence
      Command command = new InstantCommand(() -> {
         buttonBox.clearTargets();
         buttonBox.addTarget(backupTarget);
      })
      // Phase 1: Fast profile to drive to backup position
      .andThen(drivebase.startFastCustomProfileDriveToPose(
            buttonBox, 
            AutoAlgaeReefRemovalConstants.FAST_DRIVE_MAX_VEL,
            AutoAlgaeReefRemovalConstants.FAST_DRIVE_MAX_ACCEL))
      
      // Wait until we're close enough to the backup point or timeout
      .andThen(new WaitUntilCommand(() -> 
            drivebase.getDistanceToTargetPose() < AutoAlgaeReefRemovalConstants.PROXIMITY_THRESHOLD))
      
      // Add configurable wait time after backup drive
      .andThen(() -> new WaitCommand(SmartDashboard.getNumber(
            AutoAlgaeReefRemovalConstants.BACKUP_DRIVE_WAIT_KEY,
            AutoAlgaeReefRemovalConstants.DEFAULT_BACKUP_WAIT_TIME)))
      
      // Configure shooter arm based on high/low ball
      .andThen(new InstantCommand(() -> {
         if (isHighBall) {
            shooterArm.setShooterArmPreBall();
         } else {
            shooterArm.setShooterArmPreLowBall();
         }
      }))
      
      // Add configurable wait time after arm configuration
      .andThen(() -> new WaitCommand(SmartDashboard.getNumber(
            AutoAlgaeReefRemovalConstants.ARM_CONFIG_WAIT_KEY,
            AutoAlgaeReefRemovalConstants.DEFAULT_ARM_CONFIG_WAIT_TIME)))
      
      // Phase 2: Normal profile to drive to final position
      .andThen(new InstantCommand(() -> {
         buttonBox.clearTargets();
         buttonBox.addTarget(finalTarget);
      }))
      .andThen(drivebase.startCustomProfileDriveToPose(
            buttonBox, 
            AutoAlgaeReefRemovalConstants.NORMAL_DRIVE_MAX_VEL,
            AutoAlgaeReefRemovalConstants.NORMAL_DRIVE_MAX_ACCEL))
      
      // Wait until we reach the target or get close enough
      .andThen(new WaitUntilCommand(() -> drivebase.isAtTargetPose()))
      
      // Add configurable wait time after cross drive
      .andThen(() -> new WaitCommand(SmartDashboard.getNumber(
            AutoAlgaeReefRemovalConstants.CROSS_DRIVE_WAIT_KEY,
            AutoAlgaeReefRemovalConstants.DEFAULT_CROSS_DRIVE_WAIT_TIME)))
      
      // Activate shooter to grab the ball
      .andThen(shooter.shooterOutakeCommand())
      
      // Wait to ensure ball is grabbed or timeout
      .andThen(new WaitCommand(0.5))
      
      // Stop the shooter
      .andThen(shooter.shooterZeroSpeedCommand())
      
      // Pull the arm back to a safe position
      .andThen(shooterArm.shooterArmScoreLOWCommand())
      
      // Add configurable wait time for pullback
      .andThen(() -> new WaitCommand(SmartDashboard.getNumber(
            AutoAlgaeReefRemovalConstants.PULLBACK_WAIT_KEY,
            AutoAlgaeReefRemovalConstants.DEFAULT_PULLBACK_WAIT_TIME)));
      
      command.addRequirements(shooter, shooterArm, drivebase);
      return command;
   }

   // Helper method to create overloaded versions for different use cases
   public static Command autoAlgaeReefRemovalCommand(
         Shooter shooter, 
         ShooterArm shooterArm, 
         Elevator elevator,
         SwerveSubsystem drivebase,
         ButtonBox buttonBox,
         String targetFace) {
         
      // Parse target face string to determine parameters
      // Format: "AXXY" where X is face number (1-6) and Y is side (0=left, 1=right)
      int faceNumber = Integer.parseInt(targetFace.substring(1, 2));
      boolean isLeftSide = targetFace.charAt(2) == '0';
      
      // Determine if high ball based on the face configuration from constants
      boolean isHighBall = false;
      switch(faceNumber) {
         case 1: isHighBall = Constants.TargetClassConstants.isHighAlgaeA1XX; break;
         case 2: isHighBall = Constants.TargetClassConstants.isHighAlgaeA2XX; break;
         case 3: isHighBall = Constants.TargetClassConstants.isHighAlgaeA3XX; break;
         case 4: isHighBall = Constants.TargetClassConstants.isHighAlgaeA4XX; break;
         case 5: isHighBall = Constants.TargetClassConstants.isHighAlgaeA5XX; break;
         case 6: isHighBall = Constants.TargetClassConstants.isHighAlgaeA6XX; break;
      }
      
      return autoAlgaeReefRemovalCommand(shooter, shooterArm, elevator, drivebase, buttonBox, faceNumber, isHighBall, isLeftSide);
   }
}