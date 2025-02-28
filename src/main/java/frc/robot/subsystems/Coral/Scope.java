package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Scope extends SubsystemBase {
    // Flag to enable/disable vision functionality
    private boolean visionEnabled = false;
    
    // PhotonVision camera
    private PhotonCamera camera;
    
    // Target tracking variables
    private boolean hasShape = false;
    private double shapeX = 0.0;  // X position in the frame (yaw equivalent)
    private double shapeY = 0.0;  // Y position in the frame (pitch equivalent)
    private double shapeArea = 0.0;
    
    // Constants for angle conversions
    private static final double X_DEGREES_PER_UNIT = 50.0; // Degrees per normalized X unit
    private static final double Y_DEGREES_PER_UNIT = 40.0; // Degrees per normalized Y unit
    
    // Auto-adjustment sensitivity
    private static final double AUTO_ADJUST_SENSITIVITY = 1.0; // Scale factor for auto adjustments
    
    // References to subsystems for direct control - keeping only pivot and elevator
    private ShooterPivot shooterPivot;
    private Elevator elevator;
    
    // Add state tracking for continuous aiming
    private boolean continuousAimingEnabled = false;
    private int targetLevel = 0;
    private boolean isAimingStable = false;
    private double lastPivotAdjustment = 0.0;
    private double lastElevatorAdjustment = 0.0;
    private int stableAimingCount = 0;
    
    public Scope() {
        // Initialize the camera with name "scope"
        camera = new PhotonCamera("scope");
    }
    
    // Modified method to set references to only ShooterPivot (no arm)
    public void setSubsystems(ShooterPivot pivot) {
        if (pivot != null) {
            this.shooterPivot = pivot;
        }
    }
    
    // Keep the elevator setter
    public void setElevator(Elevator elevator) {
        if (elevator != null) {
            this.elevator = elevator;
        }
    }
    
    public void enableVision(boolean enable) {
        visionEnabled = enable;
        SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
    }
    
    public boolean isVisionEnabled() {
        return visionEnabled;
    }
    
    public boolean hasTarget() {
        return hasShape;
    }
    
    // Calculate the pivot angle adjustment based on shape's horizontal position
    public double calculatePivotAngleAdjustment() {
        if (!hasShape || !visionEnabled) {
            return 0.0;
        }
        
        // Convert X position to pivot adjustment
        // Positive X is right, so negative adjustment to turn right
        return -shapeX * 0.8; // Adjust this conversion factor as needed
    }
    
    // Convert normalized position (0-1) to degrees
    public double normalizedToDegreesX(double normalizedX) {
        // Map from 0-1 to -X_DEGREES_PER_UNIT/2 to +X_DEGREES_PER_UNIT/2
        return (normalizedX - 0.5) * X_DEGREES_PER_UNIT;
    }
    
    public double normalizedToDegreesY(double normalizedY) {
        // Map from 0-1 to -Y_DEGREES_PER_UNIT/2 to +Y_DEGREES_PER_UNIT/2
        return (normalizedY - 0.5) * Y_DEGREES_PER_UNIT;
    }
    
    // Modified to only calculate pivot adjustment (no arm)
    public double calculateAutoAdjustment(double pivotAngle) {
        if (!hasShape || !visionEnabled) {
            return 0.0; // No adjustment for pivot
        }
        
        // Convert normalized positions to degrees
        double xDegrees = normalizedToDegreesX(shapeX);
        
        // Calculate pivot adjustment - this is simpler as it's directly based on X
        double pivotAdjustment = -xDegrees * AUTO_ADJUST_SENSITIVITY;
        
        return pivotAdjustment;
    }
    
    // Modified method to apply auto adjustments directly to pivot only
    public void applyAutoAdjustment() {
        if (shooterPivot != null && hasShape && visionEnabled) {
            double pivotAngle = shooterPivot.shooterPivotDesiredAngle;
            double adjustment = calculateAutoAdjustment(pivotAngle);
            
            // Apply adjustment to pivot only
            shooterPivot.moveAmount(adjustment);
            
            SmartDashboard.putNumber("Auto Pivot Adjustment", adjustment);
        }
    }
    
    // NEW: Command to apply auto adjustments
    public Command applyAutoAdjustmentCommand() {
        return new InstantCommand(this::applyAutoAdjustment);
    }
    
    // NEW: Method for continuous auto tracking
    public void enableAutoTracking(boolean enable) {
        // This flag would be checked in periodic to apply continuous tracking
        SmartDashboard.putBoolean("Auto Tracking Enabled", enable);
    }
    
    // Add getters for target info
    public double getTargetArea() {
        return shapeArea;
    }
    
    // Calculate dynamic adjustments for pivot that account for target movement
    public double calculateDynamicPivotAngleAdjustment() {
        if (!hasShape || !visionEnabled) {
            return 0.0;
        }
        
        // Calculate basic adjustment
        double basicAdjustment = -shapeX * 0.8;
        
        // Add rate-based component if we have historical data
        double movementAdjustment = 0.0;
        // Would use target velocity from tracking if available
        
        return basicAdjustment + movementAdjustment;
    }
    
    // Calculate correction for manual movement - pivot only
    public double calculatePivotAngleCorrection() {
        if (!hasShape || !visionEnabled) {
            return 0.0;
        }
        
        // Smaller correction factor for manual mode
        return -shapeX * 0.4;
    }
    
    // Calculate optimal pivot angle based on targeting
    public double calculateOptimalPivotAngle() {
        if (!hasShape || !visionEnabled || shooterPivot == null) {
            return 0.0;
        }
        
        // Start with current position
        double currentAngle = shooterPivot.shooterPivotDesiredAngle;
        
        // Calculate adjustment based on target horizontal position
        double xDegrees = normalizedToDegreesX(shapeX);
        
        // Calculate optimal angle
        double optimalAngle = currentAngle - (xDegrees * 0.9);
        
        return optimalAngle;
    }
    
    // Calculate distance-based adjustments
    private double calculateDistanceBasedAngleOffset() {
        // Use target area as a proxy for distance
        // Smaller area = farther distance = more elevation needed
        if (shapeArea < 0.001) return 0.0;
        
        // Calculate base distance factor
        double distanceFactor = 0.05 / Math.max(0.001, shapeArea);
        
        // Apply reasonable limits
        return Math.min(distanceFactor, 5.0);
    }
    
    // Calculate height adjustment for elevator based on level
    public double[] calculateHeightForLevel(int level) {
        if (!hasShape || !visionEnabled) {
            return new double[] {0.0};
        }
        
        // Special case for L4: target lower position inside coral
        if (level == 3) { // Level 3 is L4 (zero-indexed)
            // Apply a downward offset to target lower in the image
            // Negative Y is higher in the image, so we add a positive value to move down
            double baseAdjustment = -shapeY * 15.0; // Scaled for elevator units
            baseAdjustment += 10.0; // Offset to target lower position
            
            // Check if pivot is centered
            if (shooterPivot != null && Math.abs(shooterPivot.shooterPivotDesiredAngle - 
                 frc.robot.Constants.ShooterPivotConstants.centerAngle) < 0.1) {
                // Additional adjustment when pivot is centered
                baseAdjustment += 5.0; // Further down adjustment for centered pivot
                SmartDashboard.putString("L4 Target", "Lower Center Position");
            }
            
            return new double[] {baseAdjustment};
        } else {
            // Base adjustment on target vertical position and level
            double baseAdjustment = -shapeY * 15.0; // Scaled for elevator units
            
            // Adjust sensitivity based on level
            double sensitivity = 1.0;
            switch (level) {
                case 2: sensitivity = 1.5; break; // L3 medium adjustment
                case 1: sensitivity = 1.2; break; // L2 small adjustment
                default: sensitivity = 1.0; break;
            }
            
            return new double[] {baseAdjustment * sensitivity};
        }
    }
    
    // Modified method to calculate pivot and elevator adjustments only (no arm)
    public double[] calculateFullAutoAdjustment() {
        if (!hasShape || !visionEnabled) {
            return new double[] {0.0, 0.0};
        }
        
        // Calculate pivot (horizontal) adjustment
        double pivotAdjustment = -shapeX * 0.8;
        
        // Calculate elevator height adjustment
        double elevatorAdjustment = -shapeY * 15.0;
        
        // Distance based corrections
        if (shapeArea > 0.001) {
            double distanceFactor = 0.05 / Math.max(0.001, shapeArea);
            // Farther targets need higher elevation for elevator
            elevatorAdjustment += Math.min(distanceFactor * 10.0, 20.0);
        }
        
        return new double[] {pivotAdjustment, elevatorAdjustment};
    }
    
    // Method to start continuous aiming mode
    public void enableContinuousAiming(int level) {
        continuousAimingEnabled = true;
        targetLevel = level;
        stableAimingCount = 0;
        isAimingStable = false;
        SmartDashboard.putBoolean("Continuous Aiming Active", true);
        SmartDashboard.putNumber("Aiming Target Level", level);
    }
    
    // Method to stop continuous aiming
    public void disableContinuousAiming() {
        continuousAimingEnabled = false;
        isAimingStable = false;
        stableAimingCount = 0;
        SmartDashboard.putBoolean("Continuous Aiming Active", false);
    }
    
    // Check if aiming has stabilized - return true when adjustments become small
    public boolean isAimingStable() {
        return isAimingStable;
    }
    
    // Apply continuous corrections during periodic if enabled
    private void applyContinuousCorrections() {
        if (!hasShape || !visionEnabled || !continuousAimingEnabled) {
            return;
        }
        
        boolean adjustmentsMade = false;
        
        // Apply pivot corrections if needed
        if (shooterPivot != null) {
            double pivotAdjustment = calculatePivotAngleAdjustment();
            if (Math.abs(pivotAdjustment) > 0.05) {
                shooterPivot.moveAmount(pivotAdjustment * 0.5); // Use smaller movements for smoother correction
                lastPivotAdjustment = pivotAdjustment;
                adjustmentsMade = true;
                SmartDashboard.putNumber("Continuous Pivot Adjustment", pivotAdjustment);
            }
        }
        
        // Apply elevator corrections if needed
        if (elevator != null) {
            double[] elevatorAdjustments = calculateHeightForLevel(targetLevel);
            if (elevatorAdjustments.length > 0 && Math.abs(elevatorAdjustments[0]) > 0.3) {
                elevator.adjustForVision(elevatorAdjustments[0] * 0.5); // Reduced for smoother movement
                lastElevatorAdjustment = elevatorAdjustments[0];
                adjustmentsMade = true;
                SmartDashboard.putNumber("Continuous Elevator Adjustment", elevatorAdjustments[0]);
            }
        }
        
        // Check if aiming has stabilized (no significant adjustments needed)
        if (!adjustmentsMade) {
            stableAimingCount++;
            // Require several stable cycles before declaring aiming complete
            if (stableAimingCount >= 5) { // About 0.1 seconds of stability
                isAimingStable = true;
                SmartDashboard.putBoolean("Aiming Stable", true);
            }
        } else {
            stableAimingCount = 0;
            isAimingStable = false;
            SmartDashboard.putBoolean("Aiming Stable", false);
        }
    }
    
    // Command to enable continuous aiming mode
    public Command startContinuousAimingCommand(int level) {
        return new InstantCommand(() -> enableContinuousAiming(level));
    }
    
    // Command to disable continuous aiming mode
    public Command stopContinuousAimingCommand() {
        return new InstantCommand(this::disableContinuousAiming);
    }
    
    @Override
    public void periodic() {
        if (visionEnabled) {
            // ...existing code...
            
            if (hasShape) {
                // ...existing code...
                
                // Apply continuous corrections if enabled
                if (continuousAimingEnabled) {
                    applyContinuousCorrections();
                }
            } else {
                // ...existing code...
                
                // Reset aiming stable if no target
                if (continuousAimingEnabled) {
                    isAimingStable = false;
                    stableAimingCount = 0;
                }
            }
        }
    }
}
