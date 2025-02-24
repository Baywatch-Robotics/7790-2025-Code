package frc.robot.subsystems;

import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public float elevatorDesiredPosition = 0;

    public boolean isInitialized = false;

    private float desiredTotalHeight;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    private RelativeEncoder m_encoder;
    
    public Elevator() {

        m_encoder = elevatorMotor.getEncoder();
        m_encoder.setPosition(0);
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
    public void setPickup() {
        elevatorDesiredPosition = ElevatorConstants.pickupPose;
    }

    // Commands
    public Command setfullElevatorRetractCommand() {
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

    public Command setElevatorPickupCommand() {

        Command command = new InstantCommand(() -> setPickup());

        isClearToIntake();

        return command;
    }

    public Command elevatorBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null) {
                switch (currentLevelSupplier.getAsInt()) {
                    case 0:
                        setL1();
                        break;
                    case 1:
                        setL2();
                        break;
                    case 2:
                        setL3();
                        break;
                    case 3:
                        setL4();
                        break;
                }
            }
        });
        return command;
    }
    public Trigger isAtHome() {
        return new Trigger(() -> m_encoder.getPosition() >= -1);
    }

    public Trigger isClearToIntake() {
        return new Trigger(() -> m_encoder.getPosition() >= ElevatorConstants.pickupPose - 20 &&
                             m_encoder.getPosition() <= ElevatorConstants.pickupPose + 20);
    }

    public Trigger isAtSetpoint() {
        return new Trigger(() -> m_encoder.getPosition() >= elevatorDesiredPosition - 5 &&
                             m_encoder.getPosition() <= elevatorDesiredPosition + 5);
    }
    public Boolean isAtSetpointBoolean() {
        return m_encoder.getPosition() >= elevatorDesiredPosition - 5 &&
               m_encoder.getPosition() <= elevatorDesiredPosition + 5;
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ElevatorConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(elevatorDesiredPosition + amount * scale, ElevatorConstants.min, ElevatorConstants.max);

        elevatorDesiredPosition = f;

        System.out.println("Elevator Desired Position: " + elevatorDesiredPosition);
    }

    @Override
    public void periodic() {

        if (!isInitialized) {
            elevatorDesiredPosition = 0;
            isInitialized = true;
        }
         //Code for inverse kinematics
        //if(!ShooterPivot.isStraight) {
        //float ang;

        //float l;

        //ang = ShooterPivot.shooterPivotDesiredAngle * (360/25);

        //l = Constants.armLength;

        //desiredTotalHeight = elevatorDesiredPosition + l * (float)Math.sin(Units.degreesToRadians(ang));


        //}
        //else{
        //desiredTotalHeight = elevatorDesiredPosition;  
        //}
        desiredTotalHeight = elevatorDesiredPosition; 

        //System.out.println(desiredTotalHeight);
        
        desiredTotalHeight = (float)MathUtil.clamp(desiredTotalHeight, ElevatorConstants.min, ElevatorConstants.max);
        
        SmartDashboard.putNumber("Elevator Desired Height", desiredTotalHeight);
        SmartDashboard.putNumber("Elevator Current Height", elevatorMotor.getEncoder().getPosition());

        isClearToIntake();
        isAtHome();
        isAtSetpoint();
        isAtSetpointBoolean();

        elevatorClosedLoopController.setReference(desiredTotalHeight, ControlType.kMAXMotionPositionControl);
    }
}