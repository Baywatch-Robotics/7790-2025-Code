package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Coral.Shooter;
import frc.robot.subsystems.Coral.ShooterArm;
import frc.robot.subsystems.Coral.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CommandFactory {

public static Command scoreL1AutomaticCommand(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, SwerveSubsystem drivebase, Elevator elevator){
        Command command = drivebase.followPath("Right to 0");
        //.andThen(pivot.setAmpScoreCommand())
        //.andThen(new WaitCommand(1.5))
        //.andThen(shooter.startAmpShooterCommand())
       // .andThen(new WaitCommand(1))
       // .andThen(shooter.shootCommand())
       // .andThen(new WaitCommand(1))
       // .andThen(shooter.stopShooterCommand())
       // .andThen(shooter.indexStopCommand());

        command.addRequirements(drivebase, algaeArm, shooter, shooterArm, shooterPivot, elevator);

        return command;
    }

    public static Command scoreL1Command(AlgaeArm algaeArm, Shooter shooter, ShooterArm shooterArm, ShooterPivot shooterPivot, SwerveSubsystem drivebase, Elevator elevator){
        Command command = algaeArm.

        command.addRequirements(drivebase, algaeArm, shooter, shooterArm, shooterPivot, elevator);

        return command;
    }
}









//}
