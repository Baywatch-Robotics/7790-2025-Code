package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTagVisionConstants;

/**
 * QuestNav - Interface to communicate with Oculus Quest for robot positioning
 */
public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0f);
  private BooleanSubscriber questIsTracking = nt4Table.getBooleanTopic("device/isTracking").subscribe(false);
  private IntegerSubscriber questTrackingLostCount = nt4Table.getIntegerTopic("device/trackingLostCounter").subscribe(0);

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();
  
  // New variables for camera calibration
  private Pose2d cameraOffsetPose = new Pose2d();
  private boolean isCalibrated = false;
  private double lastCalibrationTime = 0;
  
  // Status tracking
  private boolean isZeroed = false;
  private double zeroingTimestamp = 0;
  private static final double ZEROING_DELAY = 0.5; // Seconds to wait after zeroing command before considering it complete

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  /**
   * Gets the Quest's measured position with calibration offset applied
   * @return Calibrated Pose2d from Quest measurements
   */
  public Pose2d getPose() {
    return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), 
                      Rotation2d.fromDegrees(getOculusYaw()));
  }
  
  /**
   * Gets the Quest's measured position with camera calibration applied
   * This should be used after calibration is complete
   * @return Camera-calibrated Pose2d from Quest measurements
   */
  public Pose2d getCalibratedPose() {
    if (!isCalibrated) {
      return getPose(); // Return uncalibrated pose if not yet calibrated
    }
    
    // Apply camera calibration offset to Quest pose
    Pose2d questPose = getPose();
    Translation2d adjustedTranslation = questPose.getTranslation().plus(cameraOffsetPose.getTranslation());
    return new Pose2d(adjustedTranslation, questPose.getRotation());
  }

  /**
   * Calibrate the Quest using camera vision as reference
   * @param cameraPose The pose from camera vision system
   */
  public void calibrateWithCamera(Pose2d cameraPose) {
    if (!isZeroed) {
      SmartDashboard.putString("Quest Calibration", "Failed - Not zeroed yet");
      return;
    }
    
    // Calculate the offset between camera pose and Quest pose
    Pose2d questPose = getPose();
    Translation2d offsetTranslation = cameraPose.getTranslation().minus(questPose.getTranslation());
    cameraOffsetPose = new Pose2d(offsetTranslation, cameraPose.getRotation());
    
    isCalibrated = true;
    lastCalibrationTime = Timer.getFPGATimestamp();
    
    SmartDashboard.putNumber("Quest/Calibration X Offset", offsetTranslation.getX());
    SmartDashboard.putNumber("Quest/Calibration Y Offset", offsetTranslation.getY());
    SmartDashboard.putString("Quest Calibration", "Success at " + lastCalibrationTime);
  }

  /**
   * Recalibrate the Quest offset without zeroing
   * Use this to correct drift during a match
   * @param referencePose The known pose to calibrate against
   */
  public void recalibrate(Pose2d referencePose) {
    // Instead of compounding offsets, we'll calculate a direct mapping
    // from raw Quest pose (after zeroing) to desired reference pose
    
    // Get raw Quest pose after zeroing but before any calibration
    Pose2d rawQuestPose = getPose(); // This already has zeroing applied
    
    // Calculate the direct offset from raw Quest pose to reference pose
    Translation2d offsetTranslation = referencePose.getTranslation().minus(rawQuestPose.getTranslation());
    cameraOffsetPose = new Pose2d(offsetTranslation, new Rotation2d());
    
    isCalibrated = true;
    lastCalibrationTime = Timer.getFPGATimestamp();
    
    SmartDashboard.putNumber("Quest/Recalibration X Offset", offsetTranslation.getX());
    SmartDashboard.putNumber("Quest/Recalibration Y Offset", offsetTranslation.getY());
    SmartDashboard.putString("Quest Calibration", "Recalibrated at " + lastCalibrationTime);
  }

  /**
   * Clear any existing calibration offsets
   * This returns to using just the zeroed pose without calibration
   */
  public void clearCalibration() {
    cameraOffsetPose = new Pose2d(); // Reset to zero offset
    isCalibrated = false;
    
    SmartDashboard.putString("Quest Calibration", "Calibration cleared");
  }

  // Gets the battery percent of the Quest.
  public Double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Gets the current tracking state of the Quest. 
  public Boolean getTrackingStatus() {
    return questIsTracking.get();
  }

  // Gets the current frame count from the Quest headset.
  public Long getFrameCount() {
    return questFrameCount.get();
  }

  // Gets the number of tracking lost events since the Quest connected to the robot. 
  public Long getTrackingLostCounter() {
    return questTrackingLostCount.get();
  }

  // Returns if the Quest is connected.
  public Boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp in NT Server Time.
  public double timestamp() {
    return questTimestamp.getAtomic().serverTime;
  }

  /**
   * Zero the relative robot heading
   * Should be called at the start of the match
   */
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
    
    // Update status
    zeroingTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Zero the absolute 3D position of the robot
   * This should be called before the match starts
   */
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
    
    // Update status
    isZeroed = true;
    zeroingTimestamp = Timer.getFPGATimestamp();
    
    // Reset calibration when zeroing position
    isCalibrated = false;
    
    SmartDashboard.putString("Quest Zero Status", "Zeroed at " + zeroingTimestamp);
  }
  
  /**
   * Check if Quest zeroing has completed
   * @return true if zeroing is complete and safe to use
   */
  public boolean isZeroingComplete() {
    // Check if zeroing has been initiated and enough time has passed for it to take effect
    return isZeroed && (Timer.getFPGATimestamp() - zeroingTimestamp > ZEROING_DELAY);
  }
  
  /**
   * Check if Quest has been calibrated with camera
   * @return true if calibration has been performed
   */
  public boolean isCalibrated() {
    return isCalibrated;
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /**
   * Update SmartDashboard with Quest status information
   */
  public void updateDashboard() {
    SmartDashboard.putBoolean("Quest/Connected", connected());
    SmartDashboard.putBoolean("Quest/Tracking", getTrackingStatus());
    SmartDashboard.putNumber("Quest/Battery", getBatteryPercent());
    SmartDashboard.putBoolean("Quest/Zeroed", isZeroed);
    SmartDashboard.putBoolean("Quest/Calibrated", isCalibrated);
    
    // Display pose data
    Pose2d pose = getCalibratedPose();
    SmartDashboard.putNumber("Quest/Pose X", pose.getX());
    SmartDashboard.putNumber("Quest/Pose Y", pose.getY());
    SmartDashboard.putNumber("Quest/Pose Rot", pose.getRotation().getDegrees());
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    // Reverse the direction to fix the reversed rotation issue
    return 360 - ret;
  }

  private Pose2d getQuestNavPose() {
    // Get the raw Quest pose from network tables
    float[] questnavPosition = questPosition.get();
    
    // Swap X and Y coordinates and adjust signs to fix the 90-degree issue
    // Previous mapping was causing forward motion to appear as rightward motion
    Translation2d questTranslation = new Translation2d(questnavPosition[0], questnavPosition[2]);
    
    // Create rotation using the corrected yaw (already fixed in getOculusYaw)
    Rotation2d questRotation = Rotation2d.fromDegrees(getOculusYaw());
    
    // Create the Quest pose
    Pose2d oculusPose = new Pose2d(questTranslation, questRotation);
    
    // Transform from Quest coordinates to robot coordinates using the defined transform
    // This handles the physical mounting position/orientation of the Quest on the robot
    Pose2d robotPose = oculusPose.transformBy(AprilTagVisionConstants.ROBOT_TO_OCULUS.inverse());
    
    return robotPose;
  }

  /**
   * Set the Quest heading to match a desired rotation
   * Use this instead of zeroHeading() when you want to preserve a specific angle
   * @param desiredRotation The rotation we want the Quest to report
   */
  public void setHeadingOffset(Rotation2d desiredRotation) {
    float[] eulerAngles = questEulerAngles.get();
    // Calculate the offset needed to make current physical angle report the desired angle
    float currentRawYaw = eulerAngles[1];
    float desiredDegrees = (float)desiredRotation.getDegrees();
    
    // Normalize to 0-360 range if needed
    while (desiredDegrees < 0) {
      desiredDegrees += 360;
    }
    desiredDegrees %= 360;
    
    // Set offset so current physical angle will report the desired angle
    yaw_offset = currentRawYaw - desiredDegrees;
    
    SmartDashboard.putNumber("Quest/Set Heading To", desiredDegrees);
    SmartDashboard.putNumber("Quest/Raw Yaw", currentRawYaw);
    SmartDashboard.putNumber("Quest/Yaw Offset", yaw_offset);
  }
}
