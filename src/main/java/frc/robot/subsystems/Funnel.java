package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.Coral.Shooter;

public class Funnel extends SubsystemBase {
    
    private boolean isInitialized = false;
    public double funnelDesiredAngle;
    
    // Add variables for position smoothing (similar to ShooterArm)
    private double previousDesiredAngle; // Track previous desired angle to detect changes
    private boolean waitingForSnapReach = false; // Track if we're waiting to reach snap position
    private double snapPosition = 0; // The position we initially snap to and target
    
    private SparkMax funnelMotor = new SparkMax(FunnelConstants.ID, MotorType.kBrushless);
    private SparkClosedLoopController funnelController = funnelMotor.getClosedLoopController();
    private AbsoluteEncoder funnelEncoder = funnelMotor.getAbsoluteEncoder();
    
    // Variables for coral detection and shaking
    private boolean isShaking = false;
    private boolean coralDetected = false;
    private boolean isMonitoringForCoral = false;
    private double lastVelocity = 0;
    private double shakeStartTime = 0;
    private double baseShakePosition = 0;
    private double coralDetectionTime = 0;
    
    public Funnel() {
        funnelMotor.configure(Configs.Funnel.funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        funnelDesiredAngle = FunnelConstants.homePosition;
        previousDesiredAngle = funnelDesiredAngle; // Initialize previous angle
        snapPosition = funnelDesiredAngle; // Initialize snap position
    }
    
    // Define desired positions for the funnel
    private void setHomePosition() {
        funnelDesiredAngle = FunnelConstants.homePosition;
        isMonitoringForCoral = false;
        stopShaking();
    }
    
    private void setFullUpPosition() {
        funnelDesiredAngle = FunnelConstants.fullUpPosition;
        isMonitoringForCoral = false;
        stopShaking();
    }
    
    // New method to set funnel to pre-intake position for coral detection
    public void setPreIntakePosition() {
        funnelDesiredAngle = FunnelConstants.preIntakePosition;
        isMonitoringForCoral = true;
        coralDetected = false;
    }
    
    // Commands to move the funnel to desired positions
    public Command funnelHomeCommand() {
        return new InstantCommand(() -> setHomePosition());
    }
    
    public Command funnelFullUpCommand() {
        return new InstantCommand(() -> setFullUpPosition());
    }
    
    // New command to move to pre-intake position
    public Command funnelPreIntakeCommand() {
        return new InstantCommand(() -> setPreIntakePosition());
    }
    
    // Start shaking the funnel to help coral entry
    private void startShaking() {
        if (!isShaking) {
            isShaking = true;
            shakeStartTime = Timer.getFPGATimestamp();
            baseShakePosition = funnelEncoder.getPosition();
        }
    }
    
    // Stop shaking the funnel
    private void stopShaking() {
        isShaking = false;
    }
    
    /**
     * Update the shaking motion of the funnel
     * This creates an oscillating movement to help coral settle into position
     */
    private void updateShaking() {
        if (!isShaking) return;
        
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - shakeStartTime;
        
        // Calculate shake position using sine wave
        double oscillation = Math.sin(elapsedTime * 2 * Math.PI * FunnelConstants.shakingFrequency) 
                            * FunnelConstants.shakingAmplitude;
        
        // Set funnel position to oscillate around the base position
        funnelDesiredAngle = baseShakePosition + oscillation;
        
        // Stop shaking after max duration as a safety feature
        if (elapsedTime > FunnelConstants.shakingDuration) {
            stopShaking();
        }
    }
    
    /**
     * Detect coral impact by monitoring motor velocity and current
     */
    private void detectCoralImpact() {
        if (!isMonitoringForCoral || isShaking || coralDetected) return;
        
        // Get current velocity and current
        double currentVelocity = funnelMotor.getEncoder().getVelocity();
        
        boolean velocityImpactDetected = false;
        
        // Check for sudden velocity change if enabled
        if (FunnelConstants.USE_VELOCITY_DETECTION) {
            // Look for a sudden drop in velocity that exceeds threshold
            double velocityDelta = Math.abs(currentVelocity - lastVelocity);
            velocityImpactDetected = velocityDelta > FunnelConstants.velocityThreshold;
        }

        // If either detection method triggers
        if (velocityImpactDetected) {
            // If first detection, record the time
            if (coralDetectionTime == 0) {
                coralDetectionTime = Timer.getFPGATimestamp();
            }
            
            // Check if detection has persisted long enough
            if (Timer.getFPGATimestamp() - coralDetectionTime >= FunnelConstants.coralDetectionTime) {
                // Coral impact detected - start shaking
                coralDetected = true;
                startShaking();
            }
        } else {
            // Reset detection timer if no detection in this cycle
            coralDetectionTime = 0;
        }
        
        // Store values for next cycle
        lastVelocity = currentVelocity;
    }
    
    // Manual control method
    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }
        
        double scale = FunnelConstants.manualMultiplier;
        double newAngle = funnelDesiredAngle + amount * scale;
        funnelDesiredAngle = MathUtil.clamp(newAngle, FunnelConstants.min, FunnelConstants.max);
        
        // Stop shaking and monitoring when manual control is used
        isShaking = false;
        isMonitoringForCoral = false;
    }
    
    // Trigger for when funnel is at position
    public Trigger atPositionTrigger() {
        return new Trigger(() -> Math.abs(funnelEncoder.getPosition() - funnelDesiredAngle) < FunnelConstants.positionTolerance);
    }
    
    // Trigger for when coral is detected in funnel
    public Trigger coralDetectedTrigger() {
        return new Trigger(() -> coralDetected);
    }
    
    // Reset coral detection state (call when coral is transferred or removed)
    public void resetCoralDetection() {
        coralDetected = false;
        coralDetectionTime = 0;
        stopShaking();
    }
    
    // Command to reset coral detection state
    public Command resetCoralDetectionCommand() {
        return new InstantCommand(this::resetCoralDetection);
    }
    
    @Override
    public void periodic() {
        if (!isInitialized) {
            funnelDesiredAngle = FunnelConstants.homePosition;
            previousDesiredAngle = funnelDesiredAngle;
            snapPosition = funnelDesiredAngle;
            isInitialized = true;
        }
        
        // Apply limits
        funnelDesiredAngle = MathUtil.clamp(funnelDesiredAngle, FunnelConstants.min, FunnelConstants.max);
        
        // Process shaking logic if active
        if (isShaking) {
            updateShaking();
        } else {
            // Only detect coral if not shaking
            if (isMonitoringForCoral) {
                detectCoralImpact();
            }
        }
        
        // Get current position for smoothing operations
        double currentPosition = funnelEncoder.getPosition();
        
        // Detect if setpoint has changed significantly and not from shaking
        if (Math.abs(funnelDesiredAngle - previousDesiredAngle) > 0.001 && !isShaking) {
            // Setpoint has changed, calculate the direction of change
            double direction = Math.signum(funnelDesiredAngle - previousDesiredAngle);
            
            // Calculate the total distance from previous to desired
            double totalDistance = Math.abs(funnelDesiredAngle - previousDesiredAngle);
            
            if (totalDistance > FunnelConstants.minSmoothingDistance) {
                // Set snap position to be exactly minSmoothingDistance away from the TARGET
                snapPosition = funnelDesiredAngle - (direction * FunnelConstants.minSmoothingDistance);
            } else {
                // If already closer than the minimum smoothing distance, just use previous position
                snapPosition = previousDesiredAngle;
            }
            
            // Enter waiting state until funnel reaches snap position
            waitingForSnapReach = true;
            
            // Update the previous desired angle
            previousDesiredAngle = funnelDesiredAngle;
        }
        
        // Check if we're waiting to reach the snap position
        if (waitingForSnapReach) {
            // Calculate how close we are to the snap position
            double distanceToSnap = Math.abs(currentPosition - snapPosition);
            
            // If we're close enough, exit waiting state and begin smoothing
            if (distanceToSnap <= FunnelConstants.snapReachThreshold) {
                waitingForSnapReach = false;
            }
            
            // While waiting, just keep targeting the snap position
            // No smoothing yet
        }
        else {
            // We've reached the snap position, apply smoothing toward the final target
            double angleDifference = funnelDesiredAngle - snapPosition;
            double distToTarget = Math.abs(angleDifference);
            
            // Only apply smoothing if the distance is significant
            if (distToTarget > 0.001) {
                // Calculate an intermediate setpoint that moves toward the target
                snapPosition += angleDifference * FunnelConstants.approachSmoothingFactor;
            } else {
                // If we're essentially at the target, just set to the target exactly
                snapPosition = funnelDesiredAngle;
            }
        }
        
        // Apply limits to the snap position
        snapPosition = MathUtil.clamp(snapPosition, FunnelConstants.min, FunnelConstants.max);
        
        // Update dashboard with all relevant values
        SmartDashboard.putNumber("Funnel Desired Angle", funnelDesiredAngle);
        SmartDashboard.putNumber("Funnel Current Angle", currentPosition);
        SmartDashboard.putNumber("Funnel Target Angle", snapPosition);
        SmartDashboard.putNumber("Funnel Current Draw", funnelMotor.getOutputCurrent());
        SmartDashboard.putNumber("Funnel Velocity", funnelMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Funnel Waiting For Snap", waitingForSnapReach);
        SmartDashboard.putBoolean("Funnel Shaking", isShaking);
        SmartDashboard.putBoolean("Funnel Coral Detected", coralDetected);
        SmartDashboard.putBoolean("Funnel Monitoring For Coral", isMonitoringForCoral);
        
        // Set the motor position to the smoothed target
        funnelController.setReference(snapPosition, ControlType.kPosition);
    }
    
    /**
     * Method to handle coral loading process:
     * 1. Move to pre-intake position
     * 2. Monitor for coral impact
     * 3. Start shaking when coral detected
     * 4. Continue until shooter confirms coral is loaded
     * 
     * @param shooter Reference to the shooter subsystem to check if coral is loaded
     */
    public Command prepareForCoralCommand(Shooter shooter) {
        return funnelPreIntakeCommand()
            .andThen(new InstantCommand(() -> {
                // Reset detection state
                coralDetected = false;
                coralDetectionTime = 0;
                isMonitoringForCoral = true;
            }));
    }
    
    /**
     * Command to shake the funnel until coral is loaded into shooter
     */
    public Command shakeUntilCoralLoadedCommand(Shooter shooter) {
        return new InstantCommand(() -> {
            // If coral already detected, start shaking
            if (coralDetected) {
                startShaking();
            } else {
                // Otherwise set to pre-intake and monitor
                setPreIntakePosition();
            }
        })
        // Continue until shooter detects coral
        .until(shooter.coralLoadedTrigger())
        // Then stop shaking and return to home
        .finallyDo((interrupted) -> {
            stopShaking();
            setHomePosition();
        });
    }
}
