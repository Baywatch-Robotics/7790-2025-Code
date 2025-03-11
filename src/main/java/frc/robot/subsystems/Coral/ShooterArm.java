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
    private float previousDesiredAngle; // Track previous desired angle to detect changes
    private boolean waitingForSnapReach = false; // Track if we're waiting to reach snap position
    private float snapPosition = 0; // The position we initially snap to and target
    
    public boolean isInitialized = false;
    
    // Add flag to track if robot is in reef zone
    private boolean isInReefZone = false;
    private boolean isInReefZoneDebounce = false; // Whether we're in the debounce period after exiting reef zone
    
    // Add variable to track the arm's lowest allowed position in reef zone
    private float reefZoneMinimumAllowedAngle;

    private SparkMax shooterArmMotor = new SparkMax(ShooterArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterArmController = shooterArmMotor.getClosedLoopController();

    private AbsoluteEncoder shooterArmEncoder = shooterArmMotor.getAbsoluteEncoder();

    public ShooterArm() {

        shooterArmMotor.configure(Configs.ShooterArm.shooterArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
        previousDesiredAngle = shooterArmDesiredAngle; // Initialize previous angle
        snapPosition = shooterArmDesiredAngle; // Initialize snap position
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
    
    public Command shooterArmBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentSideSupplier = buttonBox.currentisLeftSupplier;

        Command command = new InstantCommand(() -> {

            if (currentLevelSupplier != null && currentSideSupplier != null) {
                if (currentLevelSupplier.getAsInt() == 0) {
                    setScoreLOW();
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
    
    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterArmConstants.manualMultiplier;
        float newAngle = (float)(shooterArmDesiredAngle + amount * scale);

        // Apply minimum constraints only if moving downward and in reef zone
        if ((isInReefZone || isInReefZoneDebounce) && amount < 0) {
            // Only restrict downward movement, not upward
            newAngle = Math.max(newAngle, reefZoneMinimumAllowedAngle);
        }

        // Apply the general min/max constraints
        shooterArmDesiredAngle = (float) MathUtil.clamp(newAngle, ShooterArmConstants.min, ShooterArmConstants.maxManual);
    }

    @Override
    public void periodic() {
        
        if (!isInitialized) {
            shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
            previousDesiredAngle = shooterArmDesiredAngle;
            snapPosition = shooterArmDesiredAngle;
            reefZoneMinimumAllowedAngle = ShooterArmConstants.reefZoneMinimumAngle;
            isInitialized = true;
        }
        
        isClearToElevate();
        /*
        // Handle reef zone debounce logic
        if (isInReefZoneDebounce && !isInReefZone) {
            // Check if we've waited long enough since exiting the reef zone
            double currentTime = Timer.getFPGATimestamp();
            double timeElapsed = currentTime - reefZoneExitTime;
            
            if (timeElapsed >= ShooterArmConstants.reefZoneExitDebounceTime) {
                // Debounce period is over - reset timer and flag
                isInReefZoneDebounce = false;
                reefZoneExitTime = 0;  // Reset the exit time when debounce completes
                // Reset the minimum allowed angle to the standard value
                reefZoneMinimumAllowedAngle = ShooterArmConstants.reefZoneMinimumAngle;
            }
            
            // During debounce period, act as if still in reef zone
            // Display status on dashboard
            SmartDashboard.putBoolean("Reef Zone Debounce Active", isInReefZoneDebounce);
            SmartDashboard.putNumber("Reef Zone Exit Debounce Time", timeElapsed);
        } else {
            SmartDashboard.putBoolean("Reef Zone Debounce Active", false);
            SmartDashboard.putNumber("Reef Zone Exit Debounce Time", 0);
        }
        */
        // Get current arm position for dynamic reef zone constraint and smoothing operations
        float currentPosition = (float)shooterArmEncoder.getPosition();
        
        /*
        // Update minimum allowed angle if arm is raised above the standard minimum
        if ((isInReefZone || isInReefZoneDebounce) && currentPosition >= ShooterArmConstants.reefZoneMinimumAngle) {
            // Once above the standard minimum, use that as the minimum
            reefZoneMinimumAllowedAngle = ShooterArmConstants.reefZoneMinimumAngle;
        }
        
        // Apply reef zone constraint if in reef zone OR in debounce period
        // Only apply the constraint when trying to move downward below the minimum
        if ((isInReefZone || isInReefZoneDebounce) && shooterArmDesiredAngle < reefZoneMinimumAllowedAngle) {
            // Don't allow arm to go lower than the current minimum allowed angle
            // But allow it to move upward
            shooterArmDesiredAngle = reefZoneMinimumAllowedAngle;
            SmartDashboard.putBoolean("Reef Zone Arm Constraint", true);
        } else {
            SmartDashboard.putBoolean("Reef Zone Arm Constraint", false);
        }
        
        SmartDashboard.putNumber("Reef Minimum Allowed Angle", reefZoneMinimumAllowedAngle);
        */
        // Apply general constraints
        shooterArmDesiredAngle = (float)MathUtil.clamp(shooterArmDesiredAngle, ShooterArmConstants.min, ShooterArmConstants.max);
        
        // Detect if setpoint has changed
        if (Math.abs(shooterArmDesiredAngle - previousDesiredAngle) > 0.001) {
            // Setpoint has changed, calculate the direction of change
            float direction = Math.signum(shooterArmDesiredAngle - previousDesiredAngle);
            
            // Calculate the total distance from previous to desired
            float totalDistance = Math.abs(shooterArmDesiredAngle - previousDesiredAngle);
            
            if (totalDistance > ShooterArmConstants.minSmoothingDistance) {
                // Set snap position to be exactly minSmoothingDistance away from the TARGET
                // (not from the previous position)
                snapPosition = shooterArmDesiredAngle - (direction * ShooterArmConstants.minSmoothingDistance);
            } else {
                // If already closer than the minimum smoothing distance, just go directly
                snapPosition = previousDesiredAngle;
            }
            
            // Enter waiting state until arm reaches snap position
            waitingForSnapReach = true;
            
            // Update the previous desired angle
            previousDesiredAngle = shooterArmDesiredAngle;
        }
        
        // Check if we're waiting to reach the snap position
        if (waitingForSnapReach) {
            // Calculate how close we are to the snap position
            float distanceToSnap = Math.abs(currentPosition - snapPosition);
            
            // If we're close enough, exit waiting state and begin smoothing
            if (distanceToSnap <= ShooterArmConstants.snapReachThreshold) {
                waitingForSnapReach = false;
            }
            
            // While waiting, just keep targeting the snap position
            // No smoothing yet
        }
        else {
            // We've reached the snap position, apply smoothing toward the final target
            float angleDifference = shooterArmDesiredAngle - snapPosition;
            float distToTarget = Math.abs(angleDifference);
            
            // Only apply smoothing if the distance is significant
            if (distToTarget > 0.001) {
                // Calculate an intermediate setpoint that moves toward the target
                snapPosition += angleDifference * ShooterArmConstants.approachSmoothingFactor;
            } else {
                // If we're essentially at the target, just set to the target exactly
                snapPosition = shooterArmDesiredAngle;
            }
        }
        /*
        // Apply reef zone constraint to snap position as well - only when moving down
        if ((isInReefZone || isInReefZoneDebounce) && snapPosition < reefZoneMinimumAllowedAngle) {
            snapPosition = reefZoneMinimumAllowedAngle;
        }
        */
        // Apply limits to the snap position
        snapPosition = (float)MathUtil.clamp(
            snapPosition, 
            ShooterArmConstants.min, 
            ShooterArmConstants.max
        );

        SmartDashboard.putNumber("Shooter Arm Desired Angle", shooterArmDesiredAngle);
        SmartDashboard.putNumber("Shooter Arm Current Angle", currentPosition);
        SmartDashboard.putNumber("Shooter Arm Target Angle", snapPosition);
        SmartDashboard.putBoolean("Shooter Arm Waiting For Snap", waitingForSnapReach);
        
        // Use the snap position for the actual motor control
        shooterArmController.setReference(snapPosition, ControlType.kPosition);

    }
}