package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
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
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

    public float elevatorDesiredPosition = 0;

    public boolean isInitialized = false;

    private float desiredTotalHeight;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    private RelativeEncoder m_encoder;
    
    private RobotContainer robotContainer;
    
    public Elevator(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        
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

    private void setL3L() {
        elevatorDesiredPosition = ElevatorConstants.L3LPose;
    }
    private void setL3R() {
        elevatorDesiredPosition = ElevatorConstants.L3RPose;
    }

    private void setL2L() {
        elevatorDesiredPosition = ElevatorConstants.L2LPose;
    }
    private void setL2R() {
        elevatorDesiredPosition = ElevatorConstants.L2RPose;
    }

    private void setL1() {
        elevatorDesiredPosition = ElevatorConstants.L1Pose;
    }
    public void setPickup() {
        elevatorDesiredPosition = ElevatorConstants.pickupPose;
    }
    public void setPickupPlus() {
        elevatorDesiredPosition = ElevatorConstants.pickupPose - 12;
    }
    public void setClimbPose() {
        elevatorDesiredPosition = ElevatorConstants.climbPose;
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

    public Command setElevatorL3LCommand() {
        Command command = new InstantCommand(() -> setL3L());
        return command;
    }
    public Command setElevatorL3RCommand() {
        Command command = new InstantCommand(() -> setL3R());
        return command;
    }

    public Command setElevatorL2LCommand() {
        Command command = new InstantCommand(() -> setL2L());
        return command;
    }
    public Command setElevatorL2RCommand() {
        Command command = new InstantCommand(() -> setL2R());
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
    public Command setElevatorPickupPlusCommand() {

        Command command = new InstantCommand(() -> setPickupPlus());

        return command;
    }
    public Command setElevatorClimbPoseCommand() {
        Command command = new InstantCommand(() -> setClimbPose());
        return command;
    }
    public Command elevatorBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier.getAsInt() == 0 && currentSideSupplier.getAsBoolean() == true) {
                setL1();
            } else if (currentLevelSupplier.getAsInt() == 0 && currentSideSupplier.getAsBoolean() == false) {
                setL1();
            } else if (currentLevelSupplier.getAsInt() == 1 && currentSideSupplier.getAsBoolean() == true) {
                setL2L();
            } else if (currentLevelSupplier.getAsInt() == 1 && currentSideSupplier.getAsBoolean() == false) {
                setL2R();
            } else if (currentLevelSupplier.getAsInt() == 2 && currentSideSupplier.getAsBoolean() == true) {
                setL3L();
            } else if (currentLevelSupplier.getAsInt() == 2 && currentSideSupplier.getAsBoolean() == false) {
                setL3R();
            } else if (currentLevelSupplier.getAsInt() == 3) {
                setL4();
            }
        });
        return command;
    }
    public Trigger isAtHome() {
        return new Trigger(() -> m_encoder.getPosition() >= -1);
    }
    public Trigger isRaisedTrigger() {
        return new Trigger(() -> m_encoder.getPosition() <= -1);
    }
    public Boolean isRaised() {
         return m_encoder.getPosition() <= -1;
    }

    public Trigger isClearToIntake() {
        return new Trigger(() -> m_encoder.getPosition() >= ElevatorConstants.pickupPose - 20 &&
                             m_encoder.getPosition() <= ElevatorConstants.pickupPose + 20);
    }

    public Trigger isAtSetpoint() {
        return new Trigger(() -> m_encoder.getPosition() >= elevatorDesiredPosition - 5 &&
                             m_encoder.getPosition() <= elevatorDesiredPosition + 5);
    }
    public Trigger isClearToClimbAngle() {
        return new Trigger(() -> m_encoder.getPosition() >= ElevatorConstants.climbPose - 5 &&
                             m_encoder.getPosition() <= ElevatorConstants.climbPose + 5);
    }

    public Boolean isAtSetpointBoolean() {

        if(m_encoder.getPosition() <= 5){
            return false;
        }
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
    }

    @Override
    public void periodic() {

        if (!isInitialized) {
            elevatorDesiredPosition = 0;
            isInitialized = true;
        }
        
        desiredTotalHeight = (float)MathUtil.clamp(elevatorDesiredPosition, ElevatorConstants.min, ElevatorConstants.max);
        
        SmartDashboard.putNumber("Elevator Desired Height", desiredTotalHeight);
        SmartDashboard.putNumber("Elevator Current Height", elevatorMotor.getEncoder().getPosition());

        isClearToIntake();
        isAtHome();
        isAtSetpoint();
        isAtSetpointBoolean();
        isClearToClimbAngle();
        isRaised();
        isRaisedTrigger();

        // Update drive speed based on elevator position
        robotContainer.setDriveSpeedBasedOnElevatorAndCloseness();

        elevatorClosedLoopController.setReference(desiredTotalHeight, ControlType.kMAXMotionPositionControl);
    }
}