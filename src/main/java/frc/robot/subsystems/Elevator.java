package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.ShooterPivot;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public float elevatorDesiredPosition;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    public Elevator() {

        desiredPosition = 0;


        // different positions for scoring below here

        elevatorMotor.configure(
                Configs.Elevator.elevatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    private void setFullRetract() {

        elevatorDesiredPosition = 0;
    }

    private void setL4() {
        elevatorDesiredPosition = ElevatorConstants.L4Pose;
    }

    private void setL3() {
        elevatorDesiredPosition = ElevatorConstants.L3Pose;
    }

    private void setL2() {
        elevatorDesiredPosition = ElevatorConstants.L2Pose;
    }

    private void setL1() {
        elevatorDesiredPosition = ElevatorConstants.L1Pose;
    }

    // Commands
    public Command fullRetractCommand() {
        Command command = new InstantCommand(() -> setFullRetract());
        return command;
    }

    public Command setL4Command() {
        Command command = new InstantCommand(() -> setL4());
        return command;
    }

    public Command setL3Command() {
        Command command = new InstantCommand(() -> setL3());
        return command;
    }

    public Command setL2Command() {
        Command command = new InstantCommand(() -> setL2());
        return command;
    }

    public Command setL1Command() {
        Command command = new InstantCommand(() -> setL1());
        return command;
    }

    private void moveToSetpoint() {
        elevatorClosedLoopController.setReference(desiredPosition, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void periodic() {

         //Code for inverse kinematics

        desiredPosition = ShooterPivot;


        
        //moveToSetpoint();
        
    }
}