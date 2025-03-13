package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
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
    
    // Add flag to track if robot is in reef zone
    private boolean isInReefZone = false;
    
    // Add variable to track the arm's lowest allowed position in reef zone
    private float reefZoneMinimumAllowedAngle;

    private SparkMax shooterArmMotor = new SparkMax(ShooterArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterArmController = shooterArmMotor.getClosedLoopController();

    private AbsoluteEncoder shooterArmEncoder = shooterArmMotor.getAbsoluteEncoder();
    
    // Trapezoidal motion profile objects
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ShooterArmConstants.maxVelocity, 
            ShooterArmConstants.maxAcceleration
        )
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

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

        // Apply minimum constraints only if moving downward and in reef zone
        if (isInReefZone && amount < 0) {
            // Only restrict downward movement, not upward
            newAngle = Math.max(newAngle, reefZoneMinimumAllowedAngle);
        }

        // Apply the general min/max constraints
        shooterArmDesiredAngle = (float) MathUtil.clamp(newAngle, ShooterArmConstants.min, ShooterArmConstants.maxManual);
    }
    
    /**
     * Sets whether the robot is currently in the reef zone
     * @param inReefZone true if in reef zone, false otherwise
     */
    public void setInReefZone(boolean inReefZone) {
        this.isInReefZone = inReefZone;
    }

    @Override
    public void periodic() {
        
        if (!isInitialized) {
            shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
            m_setpoint = new TrapezoidProfile.State(shooterArmEncoder.getPosition(), 0);
            reefZoneMinimumAllowedAngle = ShooterArmConstants.reefZoneMinimumAngle;
            isInitialized = true;
        }
        
        isClearToElevate();
        
        // Get current arm position for dynamic reef zone constraint
        float currentPosition = (float)shooterArmEncoder.getPosition();
        
        // Apply reef zone constraint if in reef zone
        if (isInReefZone && shooterArmDesiredAngle < reefZoneMinimumAllowedAngle) {
            // Don't allow arm to go lower than the minimum allowed angle when in reef zone
            shooterArmDesiredAngle = reefZoneMinimumAllowedAngle;
            SmartDashboard.putBoolean("Reef Zone Arm Constraint", true);
        } else {
            SmartDashboard.putBoolean("Reef Zone Arm Constraint", false);
        }
        
        // Apply general constraints
        shooterArmDesiredAngle = (float)MathUtil.clamp(shooterArmDesiredAngle, ShooterArmConstants.min, ShooterArmConstants.max);
        
        // Set goal for motion profile
        m_goal = new TrapezoidProfile.State(shooterArmDesiredAngle, 0);
        
        // Calculate next setpoint
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

        if (DriverStation.isDisabled()) {
            shooterArmDesiredAngle = (float)shooterArmEncoder.getPosition();
            m_setpoint = new TrapezoidProfile.State(shooterArmEncoder.getPosition(), 0);
        }
        
        SmartDashboard.putNumber("Shooter Arm Desired Angle", shooterArmDesiredAngle);
        SmartDashboard.putNumber("Shooter Arm Current Angle", currentPosition);
        SmartDashboard.putNumber("Shooter Arm Profile Position", m_setpoint.position);
        SmartDashboard.putNumber("Shooter Arm Profile Velocity", m_setpoint.velocity);
        SmartDashboard.putBoolean("In Reef Zone", isInReefZone);
        SmartDashboard.putNumber("Reef Minimum Allowed Angle", reefZoneMinimumAllowedAngle);
        
        // Use the profile position for the actual motor control
        shooterArmController.setReference(m_setpoint.position, ControlType.kPosition);
    }
}