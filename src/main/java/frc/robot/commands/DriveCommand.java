package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveCommand extends Command {


    private SwerveInputStream stream;
    private Pose2d pose;

    ProfiledPIDController controllerPosition;
    ProfiledPIDController controllerRotation;

    public DriveCommand(SwerveSubsystem driveSubsystem, Pose2d pose) {
        this.pose = pose;

        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize() {
        Constraints constraints = new Constraints(1.0, 1.0);
        controllerPosition = new ProfiledPIDController(0.01, 0.0, 0.0, constraints);
        controllerRotation = new ProfiledPIDController(0.01, 0.0, 0.0, constraints);

        Supplier<Pose2d> poseSupplier = () -> pose;
        stream.driveToPose(poseSupplier, controllerPosition, controllerRotation);
    }
    @Override
    public void execute() {
        if(controllerPosition.atGoal() && controllerRotation.atGoal()){
            isFinished();
        }
    }

    @Override
    public void end(boolean interrupted) {
    
       
    }
    
}
