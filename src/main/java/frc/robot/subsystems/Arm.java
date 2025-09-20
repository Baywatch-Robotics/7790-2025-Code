package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
// Restore trapezoidal profile import
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    public float ArmDesiredAngle;
    private double kDt = 0.02; // 20ms periodic loop time
    
    public boolean isInitialized = false;

    private SparkMax ArmMotor = new SparkMax(ArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController ArmController = ArmMotor.getClosedLoopController();

    private AbsoluteEncoder ArmEncoder = ArmMotor.getAbsoluteEncoder();
    
    // New trigger storage (mirrors Elevator pattern)
    private Trigger clearToDescendTrigger;
    
    // Keep ArmFeedforward controller
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        ArmConstants.kS, 
        ArmConstants.kG,
        ArmConstants.kV,
        ArmConstants.kA
    );
    
    // Update trapezoidal profile to use constants from ArmConstants
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ArmConstants.maxVelocity,
            ArmConstants.maxAcceleration
        )
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    // Conversion factors to convert between encoder units and radians
    private final double kEncoderToRadians = 2.0 * Math.PI; // Adjust this value based on your encoder's range
    
    // Reference angle for zero position (in radians)
    private final double kHorizontalReferenceRad = Math.PI / 2.0; // 90 degrees, adjust based on your setup
    
    /**
     * Converts from encoder units to radians
     */
    private double encoderToRadians(double encoderPosition) {
        return encoderPosition * kEncoderToRadians;
    }
    
    
    /**
     * Converts from encoder position to radians used by feedforward
     * (typically 0 = horizontal, positive = above horizontal)
     */
    private double encoderToFeedforwardRadians(double encoderPosition) {
        // Convert to radians then adjust for the coordinate system
        return encoderToRadians(encoderPosition) - kHorizontalReferenceRad;
    }

    public Arm() {
        ArmMotor.configure(Configs.Arm.ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ArmDesiredAngle = (float)(ArmEncoder.getPosition());
    }

    private void setScoreLOW() {
        ArmDesiredAngle = ArmConstants.scoreAngleLOW;
    }
    private void setScoreHIGH() {
        ArmDesiredAngle = ArmConstants.scoreAngleHIGH;
    }
    private void setScoreLOWBackwards() {
        ArmDesiredAngle = ArmConstants.scoreAngleLOWBackwards;
    }
    private void setScoreHIGHBackwards() {
        ArmDesiredAngle = ArmConstants.scoreAngleHIGHBackwards;
    }
    private void setPlaceLOW() {
        ArmDesiredAngle = ArmConstants.placeAngleLOW;
    }
    private void setPlaceHIGH() {
        ArmDesiredAngle = ArmConstants.placeAngleHIGH;
    }
    private void setPlaceLOWBackwards() {
        ArmDesiredAngle = ArmConstants.placeAngleLOWBackwards;
    }
    private void setPlaceHIGHBackwards() {
        ArmDesiredAngle = ArmConstants.placeAngleHIGHBackwards;
    }
    private void setPickUp() {
        ArmDesiredAngle = ArmConstants.pickUpAngle;
    }
    private void setLollipop() {
        ArmDesiredAngle = ArmConstants.lollipopAngle;
    }
    private void setoutLoad() {
        ArmDesiredAngle = ArmConstants.outLoadAngle;
    }
    private void setScoreL1() {
        ArmDesiredAngle = ArmConstants.L1Angle;
    }
    private void setScoreL1Real() {
        ArmDesiredAngle = ArmConstants.realL1Angle;
    }
    public void setAlgaeAngle() {
        ArmDesiredAngle = ArmConstants.algaeAngle;
    }
    public void setNetAngle() {
        ArmDesiredAngle = ArmConstants.netAngle;
    }
    public void setNetBackwardsAngle() {
        ArmDesiredAngle = ArmConstants.netAngleBackwards;
    }
    public void setNetPlaceAngle() {
        ArmDesiredAngle = ArmConstants.netPlaceAngle;
    }
    public void setNetPlaceBackwardsAngle() {
        ArmDesiredAngle = ArmConstants.netPlaceAngleBackwards;
    }
    public void setProcessorAngle() {
        ArmDesiredAngle = ArmConstants.processorAngle;
    }

    public Command ArmReefAlgaeCommand()
    {
        Command command = new InstantCommand(() -> setAlgaeAngle());
        return command;
    }
    public Command ArmProcessorCommand()
    {
        Command command = new InstantCommand(() -> setProcessorAngle());
        return command;
    }
    public Command ArmScoreLOWCommand()
    {
        Command command = new InstantCommand(() -> setScoreLOW());
        return command;
    }
    public Command ArmScoreHIGHCommand()
    {
        Command command = new InstantCommand(() -> this.setScoreHIGH());
        return command;
    }
    public Command ArmScoreLOWBackwardsCommand()
    {
        Command command = new InstantCommand(() -> setScoreLOWBackwards());
        return command;
    }
    public Command ArmScoreHIGHBackwardsCommand()
    {
        Command command = new InstantCommand(() -> this.setScoreHIGHBackwards());
        return command;
    }
    public Command ArmPlaceLOWCommand()
    {
        Command command = new InstantCommand(() -> setPlaceLOW());
        return command;
    }
    public Command ArmPlaceHIGHCommand()
    {
        Command command = new InstantCommand(() -> this.setPlaceHIGH());
        return command;
    }
    public Command ArmPlaceLOWBackwardsCommand()
    {
        Command command = new InstantCommand(() -> setPlaceLOWBackwards());
        return command;
    }
    public Command ArmPlaceHIGHBackwardsCommand()
    {
        Command command = new InstantCommand(() -> this.setPlaceHIGHBackwards());
        return command;
    }
    public Command ArmScoreNetCommand()
    {
        Command command = new InstantCommand(() -> setNetAngle());
        return command;
    }
    public Command ArmScoreNetBackwardsCommand()
    {
        Command command = new InstantCommand(() -> this.setNetBackwardsAngle());
        return command;
    }
    public Command ArmPlaceNetCommand()
    {
        Command command = new InstantCommand(() -> setNetPlaceAngle());
        return command;
    }
    public Command ArmPlaceNetBackwardsCommand()
    {
        Command command = new InstantCommand(() -> this.setNetPlaceBackwardsAngle());
        return command;
    }

    public Command ArmPickUpCommand()
    {
        Command command = new InstantCommand(() -> this.setPickUp());
        return command;
    }

    public Command ArmLollipopCommand()
    {
        Command command = new InstantCommand(() -> this.setLollipop());
        return command;
    }

    public Command ArmOutLoadCommand()
    {
        Command command = new InstantCommand(() -> this.setoutLoad());
        return command;
    }
    public Command ArmScoreL1Command()
    {
        Command command = new InstantCommand(() -> this.setScoreL1());
        return command;
    }

    public Command ArmScoreL1RealCommand()
    {
        Command command = new InstantCommand(() -> this.setScoreL1Real());
        return command;
    }
    
    public Command ArmBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;
        BooleanSupplier currentRotationSupplier = buttonBox.currentisForwardsSupplier;
        BooleanSupplier isAlgaeSupplier = buttonBox.isAlgaeTargetSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null && currentSideSupplier != null) {
                if (isAlgaeSupplier.getAsBoolean()) {
                    if (currentLevelSupplier.getAsInt() == 3) 
                        {if (currentRotationSupplier.getAsBoolean()) {
                        setNetAngle();
                        } else {
                        setNetBackwardsAngle();
                        }
                    } else if (currentLevelSupplier.getAsInt() == 0) {
                        setProcessorAngle();
                    } else {
                    setAlgaeAngle();
                    }
                } else {
                    // Existing coral logic
                    if (currentLevelSupplier.getAsInt() == 0) {
                        setScoreL1();
                    } else if (currentLevelSupplier.getAsInt() == 1) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setScoreLOW();
                        } else {
                            setScoreLOWBackwards();
                        }
                    } else if (currentLevelSupplier.getAsInt() == 2) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setScoreLOW();
                        } else {
                            setScoreLOWBackwards();
                        }
                    } else if (currentLevelSupplier.getAsInt() == 3) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setScoreHIGH();
                        } else {
                            setScoreHIGHBackwards();
                        }
                    }
                }
            }
        });
        return command;
    }
    
    public Command ArmPlaceBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;
        BooleanSupplier currentRotationSupplier = buttonBox.currentisForwardsSupplier;
        BooleanSupplier isAlgaeSupplier = buttonBox.isAlgaeTargetSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null && currentSideSupplier != null) {
                if (isAlgaeSupplier.getAsBoolean()) {
                    if (currentLevelSupplier.getAsInt() == 3) 
                        {if (currentRotationSupplier.getAsBoolean()) {
                        setNetPlaceAngle();
                        } else {
                        setNetPlaceBackwardsAngle();
                        }
                    }
                    } else {
                    // Existing coral logic
                    if (currentLevelSupplier.getAsInt() == 0) {
                        setScoreL1();
                    } else if (currentLevelSupplier.getAsInt() == 1) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setPlaceLOW();
                        } else {
                            setPlaceLOWBackwards();
                        }
                    } else if (currentLevelSupplier.getAsInt() == 2) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setPlaceLOW();
                        } else {
                            setPlaceLOWBackwards();
                        }
                    } else if (currentLevelSupplier.getAsInt() == 3) {
                        if (currentRotationSupplier.getAsBoolean()) {
                            setPlaceHIGH();
                        } else {
                            setPlaceHIGHBackwards();
                        }
                    }
                }
            }
        });
        return command;
    }


    public Trigger isClearToElevate() {
        return new Trigger(() -> ArmEncoder.getPosition() >= 0.5);
    }
    
    // New: trigger becomes active when arm at or above 0.25
    public Trigger isClearToDescend() {
        return new Trigger(() -> {
            boolean clear = ArmEncoder.getPosition() >= 0.25;
            SmartDashboard.putBoolean("Arm Clear To Descend", clear);
            return clear;
        });
    }
    
    /**
     * Returns a trigger that's immediately true if the queue is for level 3 or 4
     * Otherwise it checks the standard clearance condition
     * @param buttonBox the button box to check the queue from
     * @return Trigger that's either immediately true for levels 3-4 or checks clearance
     */
    public Trigger isClearToElevateBasedOnQueue(ButtonBox buttonBox) {
        return new Trigger(() -> {
            // If we have a button box and there's a target
            if (buttonBox != null && buttonBox.currentLevelSupplier != null) {
                // Get the current level from the queue
                int currentLevel = buttonBox.currentLevelSupplier.getAsInt();
                
                // Immediately return true for levels 3 and 4
                if (currentLevel == 3 || currentLevel == 4) {
                    return true;
                }
            }
            
            // For all other cases, use the standard clearance condition
            return ArmEncoder.getPosition() >= 0.5;
        });
    }

    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ArmConstants.manualMultiplier;
        float newAngle = (float)(ArmDesiredAngle + amount * scale);

        // Apply the general min/max constraints
        ArmDesiredAngle = (float) MathUtil.clamp(newAngle, ArmConstants.min, ArmConstants.maxManual);
    }
    
    @Override
    public void periodic() {
        
        if (!isInitialized) {
            ArmDesiredAngle = (float)(ArmEncoder.getPosition());
            // Restore setpoint initialization
            m_setpoint = new TrapezoidProfile.State(ArmEncoder.getPosition(), 0);
            
            isInitialized = true;
        }
        
        isClearToElevate();
        
        // Initialize new trigger once
        if (clearToDescendTrigger == null) {
            clearToDescendTrigger = isClearToDescend();
        }
        // Evaluate to refresh dashboard value
        clearToDescendTrigger.getAsBoolean();
        
        // Get current arm position for dynamic reef zone constraint
        float currentPosition = (float)ArmEncoder.getPosition();
        
        // Apply general constraints
        ArmDesiredAngle = (float)MathUtil.clamp(ArmDesiredAngle, ArmConstants.min, ArmConstants.max);
        
        // Restore trapezoidal profile calculation
        // Set goal for motion profile
        m_goal = new TrapezoidProfile.State(ArmDesiredAngle, 0);
        
        // Calculate next setpoint
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

        if (DriverStation.isDisabled()) {
            ArmDesiredAngle = (float)ArmEncoder.getPosition();
            // Restore setpoint reset
            m_setpoint = new TrapezoidProfile.State(ArmEncoder.getPosition(), 0);
        }
        
        // Convert profile positions to radians for feedforward
        double currentPositonRad = encoderToFeedforwardRadians(ArmEncoder.getPosition() - ArmConstants.feedforwardOffset);
        double currentVelocityRad = ArmEncoder.getVelocity() * kEncoderToRadians;
        
        // Calculate the feedforward output using radians
        double feedforwardOutput = armFeedforward.calculate(
            currentPositonRad,    // Position in radians (0 = horizontal)
            currentVelocityRad,    // Velocity in radians/second
            0                       // Zero acceleration for now
        );
        
        SmartDashboard.putNumber("Arm Current", ArmMotor.getOutputCurrent());
        SmartDashboard.putNumber(" Arm Desired Angle", ArmDesiredAngle);
        SmartDashboard.putNumber(" Arm Current Angle", currentPosition);
        SmartDashboard.putNumber(" Arm Feedforward", feedforwardOutput);
        SmartDashboard.putNumber(" Arm Position (rad)", currentPositonRad);
        SmartDashboard.putNumber(" Arm Velocity (rad/s)", currentVelocityRad);
        // Restore profile metrics
        SmartDashboard.putNumber(" Arm Profile Position", m_setpoint.position);
        SmartDashboard.putNumber(" Arm Profile Velocity", m_setpoint.velocity);
        
        // Use profiled position with feedforward
        ArmController.setReference(
            m_setpoint.position, 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,  // Use slot 0 for PID
            feedforwardOutput,
            ArbFFUnits.kVoltage
        );
    }

    /**
     * Command that sets  arm position based on the next target ending in '1' from the queue
     * This version uses the target from the queue without consuming it
     */
    public Command ArmBasedOnQueueCommandRight(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplierEndingIn1;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplierEndingIn1;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null && currentSideSupplier != null) {
                if (currentLevelSupplier.getAsInt() == 0) {
                    new InstantCommand();
                } else if (currentLevelSupplier.getAsInt() == 1) {
                    setScoreLOW();
                } else if (currentLevelSupplier.getAsInt() == 2) {
                    setScoreLOW();
                } else if (currentLevelSupplier.getAsInt() == 3) {
                    if (currentSideSupplier.getAsBoolean()) {
                        // Left L4
                        ArmDesiredAngle = ArmConstants.scoreAngleHIGH;
                    } else {
                        // Right L4
                        ArmDesiredAngle = ArmConstants.scoreAngleHIGH;
                    }
                }
            }
        });
        return command;
    }

    /**
     * Returns a trigger for the Right command that checks if it's clear to elevate based on target ending in '1'
     * This version uses the target ending in '1' without consuming it from the queue
     */
    public Trigger isClearToElevateBasedOnQueueRight(ButtonBox buttonBox) {
        return new Trigger(() -> {
            // If we have a button box and there's a target ending in '1'
            if (buttonBox != null && buttonBox.currentLevelSupplierEndingIn1 != null) {
                // Get the current level from the target ending in '1'
                int currentLevel = buttonBox.currentLevelSupplierEndingIn1.getAsInt();
                
                // Immediately return true for levels 3 and 4
                if (currentLevel == 3 || currentLevel == 4) {
                    return true;
                }
            }
            
            // For all other cases, use the standard clearance condition
            return ArmEncoder.getPosition() >= 0.5;
        });
    }

    /**
     * Command that sets  arm position based on the next target ending in '0' from the queue
     * This version uses the target from the queue without consuming it
     */
    public Command ArmBasedOnQueueCommandLeft(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplierEndingIn0;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplierEndingIn0;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null && currentSideSupplier != null) {
                if (currentLevelSupplier.getAsInt() == 0) {
                    new InstantCommand();
                } else if (currentLevelSupplier.getAsInt() == 1) {
                    setScoreLOW();
                } else if (currentLevelSupplier.getAsInt() == 2) {
                    setScoreLOW();
                } else if (currentLevelSupplier.getAsInt() == 3) {
                    if (currentSideSupplier.getAsBoolean()) {
                        // Left L4
                        ArmDesiredAngle = ArmConstants.scoreAngleHIGH;
                    } else {
                        // Right L4
                        ArmDesiredAngle = ArmConstants.scoreAngleHIGH;
                    }
                }
            }
        });
        return command;
    }

    /**
     * Returns a trigger for the Left command that checks if it's clear to elevate based on target ending in '0'
     * This version uses the target ending in '0' without consuming it from the queue
     */
    public Trigger isClearToElevateBasedOnQueueLeft(ButtonBox buttonBox) {
        return new Trigger(() -> {
            // If we have a button box and there's a target ending in '0'
            if (buttonBox != null && buttonBox.currentLevelSupplierEndingIn0 != null) {
                // Get the current level from the target ending in '0'
                int currentLevel = buttonBox.currentLevelSupplierEndingIn0.getAsInt();
                
                // Immediately return true for levels 3 and 4
                if (currentLevel == 3 || currentLevel == 4) {
                    return true;
                }
            }
            
            // For all other cases, use the standard clearance condition
            return ArmEncoder.getPosition() >= 0.5;
        });
    }

    // New overloaded version using queue context
    public Command ArmScoreCommand(ButtonBox buttonBox) {
        return new InstantCommand(() -> {
            if (buttonBox == null) return;

            IntSupplier levelSup = buttonBox.currentLevelSupplier;
            BooleanSupplier isForwardsSup = buttonBox.currentisForwardsSupplier;
            BooleanSupplier isAlgaeSup = buttonBox.isAlgaeTargetSupplier;

            if (levelSup == null || isForwardsSup == null || isAlgaeSup == null) return;

            int level = levelSup.getAsInt();
            boolean isForwards = isForwardsSup.getAsBoolean();
            boolean isAlgae = isAlgaeSup.getAsBoolean();

            float delta = 0f;

            if (!isAlgae) { // Coral logic
                switch (level) {
                    case 0:
                        break;
                    case 1:
                    case 2:
                        delta = (isForwards ? -ArmConstants.placeAngleLOW : ArmConstants.placeAngleLOW);
                        break;
                    case 3:
                        delta = (isForwards ? -ArmConstants.placeAngleHIGH : ArmConstants.placeAngleHIGH);
                        break;
                    default:
                        break;
                }
            } else { // Algae logic
                if (level == 3) {
                    delta = (isForwards ? -ArmConstants.netPlaceAngle : ArmConstants.netPlaceAngle);
                }
            }

            if (delta != 0f) {
                ArmDesiredAngle = (float)MathUtil.clamp(
                    ArmDesiredAngle + delta,
                    ArmConstants.min,
                    ArmConstants.max
                );
            }
        });
    }
}