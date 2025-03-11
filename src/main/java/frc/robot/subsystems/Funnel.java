package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.Algae.AlgaeArm;
import frc.robot.subsystems.Coral.Shooter;

public class Funnel extends SubsystemBase {
    
    private boolean isInitialized = false;
    public double funnelDesiredAngle;
    private double kDt = 0.02; // 20ms periodic loop time
    
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
    private boolean bypassProfilerForShaking = false; // New flag to bypass profiler during shaking
    
    // Trapezoidal motion profile objects
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            FunnelConstants.maxVelocity, 
            FunnelConstants.maxAcceleration
        )
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    
    // Reference to the AlgaeArm subsystem for safety checks
    private AlgaeArm algaeArm;
    
    public Funnel() {
        funnelMotor.configure(Configs.Funnel.funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        funnelDesiredAngle = FunnelConstants.homePosition;
    }
    
    /**
     * Set the AlgaeArm reference for safety checks
     * Must be called after construction and before using safety-related methods
     * @param algaeArm reference to the AlgaeArm subsystem
     */
    public void setAlgaeArmReference(AlgaeArm algaeArm) {
        this.algaeArm = algaeArm;
    }
    
    /**
     * Check if it's safe to move the funnel based on algae arm position
     * @return true if it's safe to move the funnel
     */
    public boolean isSafeToMove() {
        // If we don't have a reference to the algae arm, assume it's unsafe
        if (algaeArm == null) {
            return false;
        }
        
        // Use the algae arm's safety method which uses our constant
        return algaeArm.isSafeForFunnelExtension();
    }
    
    /**
     * Trigger that returns true when it's safe to move the funnel
     */
    public Trigger safeToMoveTrigger() {
        return new Trigger(this::isSafeToMove);
    }
    
    // Define desired positions for the funnel with safety checks
    private void setHomePosition() {
        // Home position is safe regardless of algae arm position
        funnelDesiredAngle = FunnelConstants.homePosition;
        isMonitoringForCoral = false;
        stopShaking();
    }
    
    private void setFullUpPosition() {
        // Only move if safe to do so
        if (isSafeToMove()) {
            funnelDesiredAngle = FunnelConstants.fullUpPosition;
        }
        
        isMonitoringForCoral = false;
        stopShaking();
    }
    
    // New method to set funnel to pre-intake position for coral detection
    public void setPreIntakePosition() {
        // Only move if safe to do so
        if (isSafeToMove()) {
            funnelDesiredAngle = FunnelConstants.preIntakePosition;
            isMonitoringForCoral = true;
            coralDetected = false;
        }
    }
    
    // Commands to move the funnel to desired positions
    public Command funnelHomeCommand() {
        return new InstantCommand(() -> setHomePosition());
    }
    
    // Command that waits until safe, then moves the funnel up
    public Command funnelFullUpCommand() {
        return new WaitUntilCommand(this::isSafeToMove)
            .andThen(new InstantCommand(() -> setFullUpPosition()));
    }
    
    // New command to move to pre-intake position with safety
    public Command funnelPreIntakeCommand() {
        return new WaitUntilCommand(this::isSafeToMove)
            .andThen(new InstantCommand(() -> setPreIntakePosition()));
    }
    
    // Start shaking the funnel to help coral entry
    private void startShaking() {
        if (!isShaking) {
            isShaking = true;
            shakeStartTime = Timer.getFPGATimestamp();
            baseShakePosition = funnelEncoder.getPosition();
            bypassProfilerForShaking = true; // Enable profiler bypass for more intense shaking
        }
    }
    
    // Stop shaking the funnel
    private void stopShaking() {
        isShaking = false;
        bypassProfilerForShaking = false; // Disable profiler bypass when shaking stops
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
    
    // Manual control method with safety check
    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }
        
        // If attempting to move toward full up position (decreasing angle), check safety
        if (amount < 0 && !isSafeToMove()) {
            // Unsafe to move up, prevent movement
            SmartDashboard.putBoolean("Funnel Move Blocked", true);
            return;
        }
        
        SmartDashboard.putBoolean("Funnel Move Blocked", false);
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
            m_setpoint = new TrapezoidProfile.State(FunnelConstants.homePosition, 0);
            isInitialized = true;
        }
        
        // Apply limits
        funnelDesiredAngle = MathUtil.clamp(funnelDesiredAngle, FunnelConstants.min, FunnelConstants.max);
        
        // Update safety status on dashboard
        SmartDashboard.putBoolean("Funnel Safe To Move", isSafeToMove());
        
        // Process shaking logic if active
        if (isShaking) {
            updateShaking();
        } else {
            // Only detect coral if not shaking
            if (isMonitoringForCoral) {
                detectCoralImpact();
            }
        }
        
        // Set goal for motion profile
        m_goal = new TrapezoidProfile.State(funnelDesiredAngle, 0);
        
        // Calculate next setpoint - but skip this when shaking with bypass enabled
        if (!bypassProfilerForShaking) {
            m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
            // Set the motor position to the trapezoidal profile setpoint
            funnelController.setReference(m_setpoint.position, ControlType.kPosition);
        } else {
            // For intense shaking, bypass the profiler and set position directly
            funnelController.setReference(funnelDesiredAngle, ControlType.kPosition);
            // Update setpoint position to avoid jerky movement when returning to profiled motion
            m_setpoint = new TrapezoidProfile.State(funnelDesiredAngle, 0);
        }
        
        // Update dashboard with all relevant values
        SmartDashboard.putNumber("Funnel Desired Angle", funnelDesiredAngle);
        SmartDashboard.putNumber("Funnel Current Angle", funnelEncoder.getPosition());
        SmartDashboard.putNumber("Funnel Target Angle", bypassProfilerForShaking ? funnelDesiredAngle : m_setpoint.position);
        SmartDashboard.putNumber("Funnel Current Draw", funnelMotor.getOutputCurrent());
        SmartDashboard.putNumber("Funnel Velocity", funnelMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Funnel Shaking", isShaking);
        SmartDashboard.putBoolean("Funnel Coral Detected", coralDetected);
        SmartDashboard.putBoolean("Funnel Monitoring For Coral", isMonitoringForCoral);
        SmartDashboard.putNumber("Funnel Profile Velocity", m_setpoint.velocity);


                // Calculate next setpoint - but skip this when shaking with bypass enabled
                if (!bypassProfilerForShaking) {
                    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
                    // Set the motor position to the trapezoidal profile setpoint
                    funnelController.setReference(m_setpoint.position, ControlType.kPosition);
                } else {
                    // For intense shaking, bypass the profiler and set position directly
                    funnelController.setReference(funnelDesiredAngle, ControlType.kPosition);
                    // Update setpoint position to avoid jerky movement when returning to profiled motion
                    m_setpoint = new TrapezoidProfile.State(funnelDesiredAngle, 0);
                }
    }
    
    /**
     * Method to handle coral loading process with safety check:
     * 1. Wait until safe to move
     * 2. Move to pre-intake position
     * 3. Monitor for coral impact
     * 4. Start shaking when coral detected
     * 5. Continue until shooter confirms coral is loaded
     * 
     * @param shooter Reference to the shooter subsystem to check if coral is loaded
     */
    public Command prepareForCoralCommand(Shooter shooter) {
        return new WaitUntilCommand(this::isSafeToMove)
            .andThen(funnelPreIntakeCommand()
                .andThen(new InstantCommand(() -> {
                    // Reset detection state
                    coralDetected = false;
                    coralDetectionTime = 0;
                    isMonitoringForCoral = true;
                })));
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
