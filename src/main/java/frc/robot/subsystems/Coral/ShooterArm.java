package frc.robot.subsystems.Coral;

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
import frc.robot.Constants.ShooterArmConstants;
import frc.robot.subsystems.ButtonBox;

public class ShooterArm extends SubsystemBase {

    public float shooterArmDesiredAngle;
    private double kDt = 0.02; // 20ms periodic loop time
    
    public boolean isInitialized = false;

    private SparkMax shooterArmMotor = new SparkMax(ShooterArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterArmController = shooterArmMotor.getClosedLoopController();

    private AbsoluteEncoder shooterArmEncoder = shooterArmMotor.getAbsoluteEncoder();
    
    // Keep ArmFeedforward controller
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        ShooterArmConstants.kS, 
        ShooterArmConstants.kG,
        ShooterArmConstants.kV,
        ShooterArmConstants.kA
    );
    
    // Update trapezoidal profile to use constants from ShooterArmConstants
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ShooterArmConstants.maxVelocity,
            ShooterArmConstants.maxAcceleration
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

    public ShooterArm() {
        shooterArmMotor.configure(Configs.ShooterArm.shooterArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
    }

    private void setScoreLOW() {
        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleLOW;
    }
    private void setScoreHIGH() {
        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleHIGH;
    }
    private void setLoad() {
        shooterArmDesiredAngle = ShooterArmConstants.loadAngle;
    }
    private void setoutLoad() {
        shooterArmDesiredAngle = ShooterArmConstants.outLoadAngle;
    }
    private void setScoreL1() {
        shooterArmDesiredAngle = ShooterArmConstants.L1Angle;
    }
    private void setClimbAngle() {
        shooterArmDesiredAngle = ShooterArmConstants.climbAngle;
    }

    // New method for ball position
    private void setBallAngle() {
        shooterArmDesiredAngle = ShooterArmConstants.ballAngle;
    }

    private void setPreBallAngle() {
        shooterArmDesiredAngle = ShooterArmConstants.preBallAngle;
    }

    private void setPreBallBelowAngle() {
        shooterArmDesiredAngle = ShooterArmConstants.preBallBelowAngle;
    }

    private void setPreLowBallAngle() {
        shooterArmDesiredAngle = ShooterArmConstants.preLowBallAngle;
    }

    public Command shooterArmScoreLOWCommand()
    {
        Command command = new InstantCommand(() -> setScoreLOW());
        return command;
    }
    public Command shooterArmScoreHIGHCommand()
    {
        Command command = new InstantCommand(() -> this.setScoreHIGH());
        return command;
    }

    public Command shooterArmLoadCommand()
    {
        Command command = new InstantCommand(() -> this.setLoad());
        return command;
    }

    public Command shooterArmOutLoadCommand()
    {
        Command command = new InstantCommand(() -> this.setoutLoad());
        return command;
    }
    public Command shooterArmScoreL1Command()
    {
        Command command = new InstantCommand(() -> this.setScoreL1());
        return command;
    }
    public Command shooterArmClimbCommand()
    {
        Command command = new InstantCommand(() -> this.setClimbAngle());
        return command;
    }
    
    // New command for ball position
    public Command shooterArmBallCommand()
    {
        Command command = new InstantCommand(() -> this.setBallAngle());
        return command;
    }

    public Command shooterArmPreBallCommand()
    {
        Command command = new InstantCommand(() -> this.setPreBallAngle());
        return command;
    }
    
    public Command shooterArmPreBallBelowCommand()
    {
        Command command = new InstantCommand(() -> this.setPreBallBelowAngle());
        return command;
    }
    
    public Command shooterArmPreLowBallCommand()
    {
        Command command = new InstantCommand(() -> this.setPreLowBallAngle());
        return command;
    }
    
    public Command shooterArmBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;

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
                        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleHIGH;
                    } else {
                        // Right L4
                        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleHIGH;
                    }
                }
            }
        });
        return command;
    }

    public Trigger isClearToElevate() {
        return new Trigger(() -> shooterArmEncoder.getPosition() >= 0.5);
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
            return shooterArmEncoder.getPosition() >= 0.5;
        });
    }

    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterArmConstants.manualMultiplier;
        float newAngle = (float)(shooterArmDesiredAngle + amount * scale);

        // Apply the general min/max constraints
        shooterArmDesiredAngle = (float) MathUtil.clamp(newAngle, ShooterArmConstants.min, ShooterArmConstants.maxManual);
    }
    
    @Override
    public void periodic() {
        
        if (!isInitialized) {
            shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
            // Restore setpoint initialization
            m_setpoint = new TrapezoidProfile.State(shooterArmEncoder.getPosition(), 0);
            
            isInitialized = true;
        }
        
        isClearToElevate();
        
        // Get current arm position for dynamic reef zone constraint
        float currentPosition = (float)shooterArmEncoder.getPosition();
        
        // Apply general constraints
        shooterArmDesiredAngle = (float)MathUtil.clamp(shooterArmDesiredAngle, ShooterArmConstants.min, ShooterArmConstants.max);
        
        // Restore trapezoidal profile calculation
        // Set goal for motion profile
        m_goal = new TrapezoidProfile.State(shooterArmDesiredAngle, 0);
        
        // Calculate next setpoint
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

        if (DriverStation.isDisabled()) {
            shooterArmDesiredAngle = (float)shooterArmEncoder.getPosition();
            // Restore setpoint reset
            m_setpoint = new TrapezoidProfile.State(shooterArmEncoder.getPosition(), 0);
        }
        
        // Convert profile positions to radians for feedforward
        double currentPositonRad = encoderToFeedforwardRadians(shooterArmEncoder.getPosition() - ShooterArmConstants.feedforwardOffset);
        double currentVelocityRad = shooterArmEncoder.getVelocity() * kEncoderToRadians;
        
        // Calculate the feedforward output using radians
        double feedforwardOutput = armFeedforward.calculate(
            currentPositonRad,    // Position in radians (0 = horizontal)
            currentVelocityRad,    // Velocity in radians/second
            0                       // Zero acceleration for now
        );
        
        SmartDashboard.putNumber("Shooter Arm Desired Angle", shooterArmDesiredAngle);
        SmartDashboard.putNumber("Shooter Arm Current Angle", currentPosition);
        SmartDashboard.putNumber("Shooter Arm Feedforward", feedforwardOutput);
        SmartDashboard.putNumber("Shooter Arm Position (rad)", currentPositonRad);
        SmartDashboard.putNumber("Shooter Arm Velocity (rad/s)", currentVelocityRad);
        // Restore profile metrics
        SmartDashboard.putNumber("Shooter Arm Profile Position", m_setpoint.position);
        SmartDashboard.putNumber("Shooter Arm Profile Velocity", m_setpoint.velocity);
        
        // Use profiled position with feedforward
        shooterArmController.setReference(
            m_setpoint.position, 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,  // Use slot 0 for PID
            feedforwardOutput,
            ArbFFUnits.kVoltage
        );
    }
}