package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.QuestNav;

import static frc.robot.Constants.QuestNavVisionConstants.QUEST_NAV_TO_ROBOT;

public class QuestNavVision extends SubsystemBase {
    private final QuestNav questNav;
    private boolean poseSetInProgress = false;
    private double poseSetStartTime = 0;
    private static final double POSE_SET_TIMEOUT = 3.0; // 3 second timeout

    public QuestNavVision() {
        this.questNav = new QuestNav();
    }

    @Override
    public void periodic() {
        // Process heartbeat requests
        questNav.processHeartbeat();
        
        // Handle cleanup of QuestNav messages
        questNav.cleanUpQuestNavMessages();

        // Update status on SmartDashboard
        SmartDashboard.putBoolean("Quest Connected", questNav.connected());
        SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent());
        
        // Monitor pose set process
        if (poseSetInProgress) {
            if (questNav.isPoseResetComplete()) {
                // Pose set is complete
                poseSetInProgress = false;
                SmartDashboard.putBoolean("Quest Pose Set Active", false);
                SmartDashboard.putString("Quest Pose Set Status", "Complete");
            } else {
                // Check for timeout
                if (Timer.getFPGATimestamp() - poseSetStartTime > POSE_SET_TIMEOUT) {
                    poseSetInProgress = false;
                    SmartDashboard.putBoolean("Quest Pose Set Active", false);
                    SmartDashboard.putString("Quest Pose Set Status", "Timeout");
                }
            }
        }
    }

    public Pair<Pose2d, Double> getPose() {
        // Get the Quest's measured position
        Pose2d questToField = questNav.getPose();
        // Transform the Quest's position to the robot's position
        Pose2d robotToField = questToField.transformBy(QUEST_NAV_TO_ROBOT.inverse());
        // Get the timestamp from the Quest and return the pose with said timestamp

        // Log position information
        SmartDashboard.putNumber("Quest Raw Pose X", questToField.getX());
        SmartDashboard.putNumber("Quest Raw Pose Y", questToField.getY());
        SmartDashboard.putNumber("Quest Raw Rotation", questToField.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Quest Corrected Pose X", robotToField.getX());
        SmartDashboard.putNumber("Quest Corrected Pose Y", robotToField.getY());
        SmartDashboard.putNumber("Quest Corrected Rotation", robotToField.getRotation().getDegrees());

        return new Pair<>(robotToField, questNav.timestamp());
    }

    public void setPose(Pose2d fieldToRobot) {
        // Log the received pose to set
        SmartDashboard.putNumber("Quest Set Pose X", fieldToRobot.getX());
        SmartDashboard.putNumber("Quest Set Pose Y", fieldToRobot.getY());
        SmartDashboard.putNumber("Quest Set Pose Rot", fieldToRobot.getRotation().getDegrees());
        
        // Transform the field relative position to the Quest's relative position
        Pose2d fieldToQuest = fieldToRobot.plus(QUEST_NAV_TO_ROBOT);
        
        // Log the transformed pose 
        SmartDashboard.putNumber("Quest Transformed Set X", fieldToQuest.getX());
        SmartDashboard.putNumber("Quest Transformed Set Y", fieldToQuest.getY());
        SmartDashboard.putNumber("Quest Transformed Set Rot", fieldToQuest.getRotation().getDegrees());
        // Set the new pose data in NetworkTables
        questNav.resetPose(fieldToQuest);
        // Make sure to flush the NetworkTables update
    }

    /**
     * Wait for a pose set operation to complete
     * @param timeoutSeconds Maximum time to wait in seconds
     * @return true if completed successfully, false if timed out
     */
    public boolean waitForPoseSetComplete(double timeoutSeconds) {
        double startTime = Timer.getFPGATimestamp();
        while (poseSetInProgress) {
            // Check for timeout
            if (Timer.getFPGATimestamp() - startTime > timeoutSeconds) {
                return false;  // Timeout occurred
            }
            Timer.delay(0.02);  // Small delay to avoid CPU hogging
        }
        return true;  // Completed successfully
    }
    
    /**
     * Force reset the pose set status (for recovery from errors)
     */
    public void forceResetPoseSetStatus() {
        poseSetInProgress = false;
        SmartDashboard.putBoolean("Quest Pose Set Active", false);
        SmartDashboard.putString("Quest Pose Set Status", "Force Reset");
    }
}