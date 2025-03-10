package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

    private double kDt = 0.02;

    public float elevatorDesiredPosition = 0;

    public boolean isInitialized = false;

    private float desiredTotalHeight;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);
    private SparkMax elevatorSlaveMotor = new SparkMax(ElevatorConstants.slaveID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
    
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(.5, .5));

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();


    private RelativeEncoder m_encoder;
    
    private RobotContainer robotContainer;
    
    // Store triggers as instance variables
    private Trigger atSetpointTrigger;
    private Trigger atHomeTrigger;
    private Trigger raisedTrigger;
    private Trigger clearToIntakeTrigger;
    private Trigger clearToClimbAngleTrigger;
    
    // Additional triggers for more precise height detection
    private Trigger slightlyRaisedTrigger;
    private Trigger partiallyRaisedTrigger;
    private Trigger midRaisedTrigger;
    private Trigger fullyRaisedTrigger;

    public Elevator(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        
        m_encoder = elevatorMotor.getEncoder();
        m_encoder.setPosition(0);
        

        elevatorMotor.configure(
                Configs.Elevator.elevatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        

        elevatorSlaveMotor.configure(
                Configs.Elevator.elevatorSlaveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void setFullRetract() {
        elevatorDesiredPosition = ElevatorConstants.downPosition;
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
        elevatorDesiredPosition = ElevatorConstants.pickupPose - 2;
    }
    public void setClimbPose() {
        elevatorDesiredPosition = ElevatorConstants.climbPose;
    }
    
    // New methods for ball positions
    /*
    public void setHighBall() {
        elevatorDesiredPosition = ElevatorConstants.highBallPose;
    }
    
    public void setLowBall() {
        elevatorDesiredPosition = ElevatorConstants.lowBallPose;
    }
    */
    
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
    
    // New commands for ball positions
    /*
    public Command setElevatorHighBallCommand() {
        Command command = new InstantCommand(() -> setHighBall());
        return command;
    }
    
    public Command setElevatorLowBallCommand() {
        Command command = new InstantCommand(() -> setLowBall());
        return command;
    }
    */
    
    public Command elevatorBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier.getAsInt() == 0 && currentSideSupplier.getAsBoolean() == true) {
                setFullRetract();
            } else if (currentLevelSupplier.getAsInt() == 0 && currentSideSupplier.getAsBoolean() == false) {
                setFullRetract();
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
        return new Trigger(() -> {
            boolean atHome = m_encoder.getPosition() >= -1;
            SmartDashboard.putBoolean("Elevator At Home", atHome);
            return atHome;
        });
    }
    public Trigger isRaisedTrigger() {
        return new Trigger(() -> {
            boolean raised = m_encoder.getPosition() <= -1;
            SmartDashboard.putBoolean("Elevator Is Raised", raised);
            return raised;
        });
    }
    public Boolean isRaised() {
         return m_encoder.getPosition() <= -1;
    }

    public Trigger isClearToIntake() {
        return new Trigger(() -> {
            boolean clearToIntake = m_encoder.getPosition() >= ElevatorConstants.pickupPose - ElevatorConstants.INTAKE_CLEARANCE_MARGIN &&
                                    m_encoder.getPosition() <= ElevatorConstants.pickupPose + ElevatorConstants.INTAKE_CLEARANCE_MARGIN;
            SmartDashboard.putBoolean("Elevator Clear To Intake", clearToIntake);
            return clearToIntake;
        });
    }
    
    // Direct method to check if elevator is at intake position
    public boolean isAtIntakePosition() {
        return m_encoder.getPosition() >= ElevatorConstants.pickupPose - ElevatorConstants.APPROACHING_SETPOINT_TOLERANCE &&
               m_encoder.getPosition() <= ElevatorConstants.pickupPose + ElevatorConstants.APPROACHING_SETPOINT_TOLERANCE;
    }

    public Trigger isAtSetpoint() {
        return new Trigger(() -> {
            boolean atSetpoint = m_encoder.getPosition() >= elevatorDesiredPosition - ElevatorConstants.STANDARD_SETPOINT_TOLERANCE &&
                                 m_encoder.getPosition() <= elevatorDesiredPosition + ElevatorConstants.STANDARD_SETPOINT_TOLERANCE;
            // Log the values to help debug
            SmartDashboard.putBoolean("Elevator At Setpoint", atSetpoint);
            SmartDashboard.putNumber("Elevator Position Difference", m_encoder.getPosition() - elevatorDesiredPosition);
            return atSetpoint;
        });
    }
    public Trigger isClearToClimbAngle() {
        return new Trigger(() -> {
            boolean clearToClimb = m_encoder.getPosition() >= ElevatorConstants.climbPose - 5 &&
                                   m_encoder.getPosition() <= ElevatorConstants.climbPose + 5;
            SmartDashboard.putBoolean("Elevator Clear To Climb", clearToClimb);
            return clearToClimb;
        });
    }

    public Boolean isAtSetpointBoolean() {

        if(m_encoder.getPosition() <= ElevatorConstants.HOME_POSITION_THRESHOLD){
            return false;
        }
        return m_encoder.getPosition() >= elevatorDesiredPosition - ElevatorConstants.HOME_POSITION_THRESHOLD &&
               m_encoder.getPosition() <= elevatorDesiredPosition + ElevatorConstants.HOME_POSITION_THRESHOLD;
    }
    
    // New methods to check elevator height status
    public Trigger isSlightlyRaisedTrigger() {
        return new Trigger(() -> {
            boolean slightlyRaised = m_encoder.getPosition() <= ElevatorConstants.SLIGHTLY_RAISED_THRESHOLD;
            SmartDashboard.putBoolean("Elevator Slightly Raised", slightlyRaised);
            return slightlyRaised;
        });
    }
    
    public Trigger isPartiallyRaisedTrigger() {
        return new Trigger(() -> {
            boolean partiallyRaised = m_encoder.getPosition() <= ElevatorConstants.PARTIALLY_RAISED_THRESHOLD;
            SmartDashboard.putBoolean("Elevator Partially Raised", partiallyRaised);
            return partiallyRaised;
        });
    }
    
    public Trigger isMidRaisedTrigger() {
        return new Trigger(() -> {
            boolean midRaised = m_encoder.getPosition() <= ElevatorConstants.MID_RAISED_THRESHOLD;
            SmartDashboard.putBoolean("Elevator Mid Raised", midRaised);
            return midRaised;
        });
    }
    
    public Trigger isFullyRaisedTrigger() {
        return new Trigger(() -> {
            boolean fullyRaised = m_encoder.getPosition() <= ElevatorConstants.FULLY_RAISED_THRESHOLD;
            SmartDashboard.putBoolean("Elevator Fully Raised", fullyRaised);
            return fullyRaised;
        });
    }
    
    // New boolean methods for direct checking
    public boolean isSlightlyRaised() {
        return m_encoder.getPosition() <= ElevatorConstants.SLIGHTLY_RAISED_THRESHOLD;
    }
    
    public boolean isPartiallyRaised() {
        return m_encoder.getPosition() <= ElevatorConstants.PARTIALLY_RAISED_THRESHOLD;
    }
    
    public boolean isMidRaised() {
        return m_encoder.getPosition() <= ElevatorConstants.MID_RAISED_THRESHOLD;
    }
    
    public boolean isFullyRaised() {
        return m_encoder.getPosition() <= ElevatorConstants.FULLY_RAISED_THRESHOLD;
    }
    
    // Get elevator height category as an enum for easier state management
    public enum ElevatorHeight {
        LOWERED,
        SLIGHTLY_RAISED,
        PARTIALLY_RAISED,
        MID_RAISED,
        FULLY_RAISED
    }
    
    public ElevatorHeight getElevatorHeightCategory() {
        double position = m_encoder.getPosition();
        
        if (position <= ElevatorConstants.FULLY_RAISED_THRESHOLD) {
            return ElevatorHeight.FULLY_RAISED;
        } else if (position <= ElevatorConstants.MID_RAISED_THRESHOLD) {
            return ElevatorHeight.MID_RAISED;
        } else if (position <= ElevatorConstants.PARTIALLY_RAISED_THRESHOLD) {
            return ElevatorHeight.PARTIALLY_RAISED;
        } else if (position <= ElevatorConstants.SLIGHTLY_RAISED_THRESHOLD) {
            return ElevatorHeight.SLIGHTLY_RAISED;
        } else {
            return ElevatorHeight.LOWERED;
        }
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

        
        

        SmartDashboard.putNumber("Elevator Desired Height", desiredTotalHeight);
        SmartDashboard.putNumber("Elevator Current Height", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putString("Elevator Height Category", getElevatorHeightCategory().toString());
        
        SmartDashboard.putNumber("Elevator Desired Power", elevatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Slave Power", elevatorSlaveMotor.getAppliedOutput());

        SmartDashboard.putNumber("Elevator Velocity", elevatorMotor.getEncoder().getVelocity());


        // Initialize triggers once
        if (atSetpointTrigger == null) {
            atSetpointTrigger = isAtSetpoint();
        }
        
        if (atHomeTrigger == null) {
            atHomeTrigger = isAtHome();
        }
        
        if (raisedTrigger == null) {
            raisedTrigger = isRaisedTrigger();
        }
        
        if (clearToIntakeTrigger == null) {
            clearToIntakeTrigger = isClearToIntake();
        }
        
        if (clearToClimbAngleTrigger == null) {
            clearToClimbAngleTrigger = isClearToClimbAngle();
        }
        
        // Initialize the new triggers
        if (slightlyRaisedTrigger == null) {
            slightlyRaisedTrigger = isSlightlyRaisedTrigger();
        }
        
        if (partiallyRaisedTrigger == null) {
            partiallyRaisedTrigger = isPartiallyRaisedTrigger();
        }
        
        if (midRaisedTrigger == null) {
            midRaisedTrigger = isMidRaisedTrigger();
        }
        
        if (fullyRaisedTrigger == null) {
            fullyRaisedTrigger = isFullyRaisedTrigger();
        }
        
        // Evaluate all triggers
        clearToIntakeTrigger.getAsBoolean();
        atHomeTrigger.getAsBoolean();
        atSetpointTrigger.getAsBoolean();
        clearToClimbAngleTrigger.getAsBoolean();
        raisedTrigger.getAsBoolean();
        slightlyRaisedTrigger.getAsBoolean();
        partiallyRaisedTrigger.getAsBoolean();
        midRaisedTrigger.getAsBoolean();
        fullyRaisedTrigger.getAsBoolean();

        // Update drive speed based on elevator position
        robotContainer.setDriveSpeedBasedOnElevatorAndCloseness();


        desiredTotalHeight = (float)MathUtil.clamp(elevatorDesiredPosition, ElevatorConstants.min, ElevatorConstants.max);

        m_goal = new TrapezoidProfile.State(desiredTotalHeight, 0);

        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);


        elevatorClosedLoopController.setReference(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.FFPercent, ArbFFUnits.kPercentOut);
        
    }
    
    // Use these methods to access the stored trigger instances
    public Trigger getIsAtHomeTrigger() {
        return atHomeTrigger;
    }
    
    public Trigger getIsRaisedTrigger() {
        return raisedTrigger;
    }
    
    public Trigger getIsClearToIntakeTrigger() {
        return clearToIntakeTrigger;
    }
    
    public Trigger getIsAtSetpointTrigger() {
        return atSetpointTrigger;
    }
    
    public Trigger getIsClearToClimbAngleTrigger() {
        return clearToClimbAngleTrigger;
    }
    
    // Getters for the new trigger instances
    public Trigger getIsSlightlyRaisedTrigger() {
        return slightlyRaisedTrigger;
    }
    
    public Trigger getIsPartiallyRaisedTrigger() {
        return partiallyRaisedTrigger;
    }
    
    public Trigger getIsMidRaisedTrigger() {
        return midRaisedTrigger;
    }
    
    public Trigger getIsFullyRaisedTrigger() {
        return fullyRaisedTrigger;
    }
}