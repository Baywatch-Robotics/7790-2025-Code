package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private double desiredPosition;

    private float elevatorMax;
    private float elevatorMin;

    // List different poses here
    private float L4Pose;
    private float L3Pose;
    private float L2Pose;
    private float L1Pose;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    public Elevator() {

        desiredPosition = 0;

        L4Pose = ElevatorConstants.L4Pose;
        L3Pose = ElevatorConstants.L3Pose;
        L2Pose = ElevatorConstants.L2Pose;
        L1Pose = ElevatorConstants.L1Pose;

        elevatorMax = ElevatorConstants.Max;
        elevatorMin = ElevatorConstants.Min;

        // different positions for scoring below here

        elevatorMotor.configure(
                Configs.Elevator.elevatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void setDesiredPosition(final float desiredPose) {

        float newDesiredPose = (float) MathUtil.clamp(desiredPose, elevatorMin, elevatorMax);

        this.desiredPosition = (newDesiredPose);
    }

    public void setFullRetract() {

        this.desiredPosition = 0;
    }

    public void setL4() {
        setDesiredPosition(L4Pose);
    }

    public void setL3() {
        setDesiredPosition(L3Pose);
    }

    public void setL2() {
        setDesiredPosition(L2Pose);
    }

    public void setL1() {
        setDesiredPosition(L1Pose);
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
        moveToSetpoint();
    }
}