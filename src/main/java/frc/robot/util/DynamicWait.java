package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WaitTimeConstants;

/**
 * Utility class for managing dynamically configurable wait times.
 * Wait times are stored in SmartDashboard for easy tuning and can be
 * accessed at runtime by name.
 */
public class DynamicWait {
    
    /**
     * Initializes all wait times with their default values.
     * Call this method in robotInit() to ensure all wait times are properly set.
     */
    public static void initializeWaitTimes() {
        // Simply initialize all wait times with default values
        SmartDashboard.putNumber(WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + WaitTimeConstants.INITIAL_PLACEMENT_TIME, 
                                WaitTimeConstants.DEFAULT_INITIAL_PLACEMENT_TIME);
        SmartDashboard.putNumber(WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + WaitTimeConstants.FIRST_BALL_TIME, 
                                WaitTimeConstants.DEFAULT_FIRST_BALL_TIME);
        SmartDashboard.putNumber(WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + WaitTimeConstants.SECOND_BALL_TIME, 
                                WaitTimeConstants.DEFAULT_SECOND_BALL_TIME);
        SmartDashboard.putNumber(WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + WaitTimeConstants.THIRD_BALL_TIME, 
                                WaitTimeConstants.DEFAULT_THIRD_BALL_TIME);
    }
    
    /**
     * Creates a wait command that will wait for the duration specified by the named wait time.
     * 
     * @param waitTimeName The name of the wait time to use
     * @return A command that will wait for the specified duration
     */
    public static Command waitCommand(String waitTimeName) {
        // Read directly from SmartDashboard when creating the command
        String key = WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + waitTimeName;
        double waitTime = SmartDashboard.getNumber(key, 0.0);
        return new WaitCommand(waitTime);
    }
    
    /**
     * Creates a dynamic wait command that will evaluate the wait time when it executes.
     * 
     * @param waitTimeName The name of the wait time to use
     * @return A command that gets the wait time when executed
     */
    public static Command dynamicWaitCommand(String waitTimeName) {
        return new Command() {
            private double endTime;
            
            @Override
            public void initialize() {
                // Read directly from SmartDashboard at execution time
                String key = WaitTimeConstants.WAIT_TIMES_KEY_PREFIX + waitTimeName;
                double waitTime = SmartDashboard.getNumber(key, 0.0);
                endTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + waitTime;
            }
            
            @Override
            public boolean isFinished() {
                return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() >= endTime;
            }
        };
    }
}
