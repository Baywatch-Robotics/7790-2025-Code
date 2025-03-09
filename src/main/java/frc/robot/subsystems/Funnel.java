package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
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
import frc.robot.Constants.FunnelConstants;

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
    
    public Funnel() {
        funnelMotor.configure(Configs.Funnel.funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        funnelDesiredAngle = FunnelConstants.homePosition;
        previousDesiredAngle = funnelDesiredAngle; // Initialize previous angle
        snapPosition = funnelDesiredAngle; // Initialize snap position
    }
    
    // Define desired positions for the funnel
    private void setHomePosition() {
        funnelDesiredAngle = FunnelConstants.homePosition;
    }
    
    private void setFullUpPosition() {
        funnelDesiredAngle = FunnelConstants.fullUpPosition;
    }
    
    // Commands to move the funnel to desired positions
    public Command funnelHomeCommand() {
        return new InstantCommand(() -> setHomePosition());
    }
    
    public Command funnelFullUpCommand() {
        return new InstantCommand(() -> setFullUpPosition());
    }
    
    // Manual control method
    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }
        
        double scale = FunnelConstants.manualMultiplier;
        double newAngle = funnelDesiredAngle + amount * scale;
        funnelDesiredAngle = MathUtil.clamp(newAngle, FunnelConstants.min, FunnelConstants.max);
    }
    
    // Trigger for when funnel is at position
    public Trigger atPositionTrigger() {
        return new Trigger(() -> Math.abs(funnelEncoder.getPosition() - funnelDesiredAngle) < FunnelConstants.positionTolerance);
    }
    
    @Override
    public void periodic() {
        if (!isInitialized) {
            funnelDesiredAngle = (double)(funnelEncoder.getPosition());
            previousDesiredAngle = funnelDesiredAngle;
            snapPosition = funnelDesiredAngle;
            isInitialized = true;
        }
        
        // Apply limits
        funnelDesiredAngle = MathUtil.clamp(funnelDesiredAngle, FunnelConstants.min, FunnelConstants.max);
        
        // Get current position for smoothing operations
        double currentPosition = funnelEncoder.getPosition();
        
        // Detect if setpoint has changed significantly
        if (Math.abs(funnelDesiredAngle - previousDesiredAngle) > 0.001) {
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
        SmartDashboard.putBoolean("Funnel Waiting For Snap", waitingForSnapReach);
        
        // Set the motor position to the smoothed target
        funnelController.setReference(snapPosition, ControlType.kPosition);
    }
}
