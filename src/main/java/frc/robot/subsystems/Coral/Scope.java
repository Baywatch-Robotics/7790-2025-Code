package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double shapeWidth = 0.0;
    private double shapeHeight = 0.0;
    
    // Constants for colored shape detection
    private static final int PURPLE_POLE_PIPELINE_INDEX = 0;  // Set correct index for purple pole detection
    
    public Scope() {
        // Initialize the camera with name "scope"
        camera = new PhotonCamera("scope");
        // Set to the pipeline that detects the purple pole/shapes
        camera.setPipelineIndex(PURPLE_POLE_PIPELINE_INDEX);
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
    
    // Calculate the arm angle adjustment based on shape's vertical position
    public double calculateArmAngleAdjustment() {
        if (!hasShape || !visionEnabled) {
            return 0.0;
        }
        
        // Convert Y position to arm angle adjustment
        // Negative because higher in frame (lower Y value) means aim higher
        return -shapeY * 0.5; // Adjust this conversion factor as needed
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
    
    @Override
    public void periodic() {
        if (visionEnabled) {
            // Query the latest result from PhotonVision
            PhotonPipelineResult result = camera.getLatestResult();
            
            // Check if the pipeline has any shapes detected
            hasShape = result.hasTargets();
            
            if (hasShape) {
                // Get the best match (largest or closest colored shape)
                PhotonTrackedTarget shape = result.getBestTarget();
                
                // Extract shape information - using standard PhotonVision methods
                // but interpreting the data as shape positions rather than target poses
                shapeX = shape.getYaw();  // Use yaw as X position in image
                shapeY = shape.getPitch(); // Use pitch as Y position in image
                shapeArea = shape.getArea();
                
                // If your pipeline provides width and height, you can get them from target bounds
                // or from custom vendor-specific shape detection data
                
                // Put the data on SmartDashboard
                SmartDashboard.putNumber("Shape X Position", shapeX);
                SmartDashboard.putNumber("Shape Y Position", shapeY);
                SmartDashboard.putNumber("Shape Area", shapeArea);
                SmartDashboard.putNumber("Arm Angle Adjustment", calculateArmAngleAdjustment());
                SmartDashboard.putNumber("Pivot Angle Adjustment", calculatePivotAngleAdjustment());
                SmartDashboard.putBoolean("Purple Pole Detected", true);
            } else {
                // Reset values when no shape is found
                SmartDashboard.putNumber("Shape X Position", 0.0);
                SmartDashboard.putNumber("Shape Y Position", 0.0);
                SmartDashboard.putNumber("Shape Area", 0.0);
                SmartDashboard.putNumber("Arm Angle Adjustment", 0.0);
                SmartDashboard.putNumber("Pivot Angle Adjustment", 0.0);
                SmartDashboard.putBoolean("Purple Pole Detected", false);
            }
        }
    }
}
