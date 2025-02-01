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
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Coral.ShooterPivot;

public class Elevator extends SubsystemBase {

    public float elevatorDesiredPosition;

    private float desiredTotalHeight;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    public Elevator() {

        elevatorDesiredPosition = 0;

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
    public Command fullElevatorRetractCommand() {
        Command command = new InstantCommand(() -> setFullRetract());
        return command;
    }

    public Command setElevatorL4Command() {
        Command command = new InstantCommand(() -> setL4());
        return command;
    }

    public Command setElevatorL3Command() {
        Command command = new InstantCommand(() -> setL3());
        return command;
    }

    public Command setElevatorL2Command() {
        Command command = new InstantCommand(() -> setL2());
        return command;
    }

    public Command setElevatorL1Command() {
        Command command = new InstantCommand(() -> setL1());
        return command;
    }

        public void moveAmount(final float amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ElevatorConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(elevatorDesiredPosition + amount * scale, ElevatorConstants.min, ElevatorConstants.max);

        elevatorDesiredPosition = f;
    }

    @Override
    public void periodic() {

         //Code for inverse kinematics
        if(!ShooterPivot.isStraight) {
        float ang;

        float l;

        ang = ShooterPivot.shooterPivotDesiredAngle * (360/25);

        l = Constants.armLength;

        desiredTotalHeight = elevatorDesiredPosition + l * (float)Math.sin(ang);


        }
        else{
        desiredTotalHeight = elevatorDesiredPosition;  
        }

        System.out.println(desiredTotalHeight);
        
        desiredTotalHeight = (float)MathUtil.clamp(desiredTotalHeight, ElevatorConstants.min, ElevatorConstants.max);
        
        //elevatorClosedLoopController.setReference(desiredTotalHeight, ControlType.kMAXMotionPositionControl);
    }
}