// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final float ROBOT_MASS = (float) Units.lbsToKilograms(115); 
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final float LOOP_TIME  = 0.13f; //s, 20ms + 110ms sprk max velocity lag
  public static final float MAX_SPEED  = (float) Units.feetToMeters(22.1);
  public static final float WHEEL_LOCK_TIME = 10;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final float slowSpeedClamp = 0.1f;
  public static final float mediumSpeedClamp = 0.2f;

  public static final float DEADBAND = 0.075f;

  public static final float armLength = (float)Units.inchesToMeters(0);
    


  public static final class AlgaeArmConstants{
    public static final int ID = 14;

    public static final float stowedUpAngle = 0.305f;
    public static final float straightOutAngle = 0.535f;
    public static final float groundIntakeAngle = 0.42f;

    public static final float holdAngle = 0.34f;

    // Current threshold for detecting algae
    public static final float currentThreshold = 15.0f;

    public static final float angleOffset = 0.75f;

    public static final float max = 0.535f;
    public static final float min = 0.28f;

    public static final float manualMultiplier = 0.05f;


    public static final float P = 2f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 50;
    public static final float maxAcceleration = 50;
    public static final float allowedClosedLoopError = .005f;
  }

    public static final class AlgaeShooterConstants{
    public static final int ID = 16;
    public static final float intake = 0.25f;
    public static final float outake = -0.4f;
    public static final float currentThreshold = 10f; // Current threshold for detecting algae
    
    // Trigger control constants
    public static final float triggerThreshold = 0.1f;    // Minimum trigger value to activate
    public static final float maxTriggerIntake = 0.5f;    // Maximum intake speed with trigger (left)
    public static final float maxTriggerOutake = -0.6f;   // Maximum outake speed with trigger (right)
    
    public static final float DEBOUNCE_TIME = 1.0f; // Time to debounce current threshold (seconds)
  }
  public static final class ShooterConstants{
    public static final int ID = 19;
    public static final float intake = 0.15f;
    public static final float outake = -.50f;
    public static final float currentThreshold = 15;
    public static final float DEBOUNCE_TIME = 0.25f; // Time to debounce current threshold (seconds)
  }

  public static final class ShooterArmConstants{
    public static final int ID = 15;

    public static final float angleOffset = .83f;

    public static final float manualMultiplier = -0.005f;

    public static final float min = 0.252f;
    //public static final float max = .9f;
    public static final float max = 0.698f;

    

    public static final float maxManual = 0.698f;
    //public static final float maxManual = .9f;
    
    public static final float climbAngle = 0.698f; //Larger than this in reality

    public static final float scoreAngleLOW = 0.698f;
    public static final float scoreAngleHIGH = 0.64f;
    // Minimum angle allowed in reef zone (prevents arm from going too low in reef)
    public static final float reefZoneMinimumAngle = 0.643f; // This should be between ballAngle and scoreAngleHIGH
    // Debounce time when exiting reef zone (seconds)
    public static final float reefZoneExitDebounceTime = 0.5f;

    public static final float loadAngle = 0.27f;
    public static final float outLoadAngle = 0.3f;

    
    // Ball position angle
    public static final float preBallAngle = 0.634489f;
    public static final float ballAngle = 0.60072f;

    public static final float L1Angle = 0.5f;

    public static final float P = 6f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    //public static final float maxVelocity = 1000;
    //public static final float maxAcceleration = 200;

    public static final float maxVelocity = 50;
    public static final float maxAcceleration = 50;


    public static final float allowedClosedLoopError = 0.005f;

    public static final float minSmoothingDistance = 1.0f;  // Minimum angle difference to trigger smoothing
    public static final float approachSmoothingFactor = 0.075f;  // 0-1 value: smaller = smoother/slower approach
    public static final float snapReachThreshold = 0.05f;  // How close arm must get to snap position before smoothing begins
  }

  public static final class ClimberConstants{

    public static final int ID = 18;

    public static final float extendSpeed = 1;
    public static final float retractSpeed = -1f;
  }
  
  public static final class ElevatorConstants{
    
    public static final int ID = 13;
    
    public static final int slaveID = 17;
    



    public static final float manualMultiplier = .5f;
    //public static final float manualMultiplier = .2f; //For Testing

    //Gray Spools
    //public static final float min = -44f;
    public static final float min = -48.1287f;
    public static final float max = 0.0f;

    // elevator set positions
    public static final float L4Pose = -48.1287f;
    public static final float L3LPose = -24;
    public static final float L3RPose = -24f
    ;
    public static final float L2RPose = -9f;
    public static final float L2LPose = -9f;
    public static final float L1Pose = -0;
    public static final float pickupPose = -23.856016f;
    public static final float climbPose = -0;

    //-6.296857f WHEN TO DISABLE FEED FORWARD
    
    public static final float downPosition = -7f;
    
    // Ball pickup positions
     public static final float highBallPose = -38.39914f;
     public static final float lowBallPose = -25.20623f;
    
    // New elevator height thresholds for speed control
    public static final float SLIGHTLY_RAISED_THRESHOLD = -10f;
    public static final float PARTIALLY_RAISED_THRESHOLD = -20f;
    public static final float MID_RAISED_THRESHOLD = -30f;
    public static final float FULLY_RAISED_THRESHOLD = -40f;
    
    // Elevator setpoint tolerance constants
    public static final float STANDARD_SETPOINT_TOLERANCE = 2.0f;
    public static final float NEAR_SETPOINT_TOLERANCE = 5.0f;
    public static final float APPROACHING_SETPOINT_TOLERANCE = 8f;
    
    // Home position threshold
    public static final float HOME_POSITION_THRESHOLD = 5.0f;
    
    // Intake clearance thresholds
    public static final float INTAKE_CLEARANCE_MARGIN = 5.0f;
    
    public static final float P = 0.75f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;

    public static final float FFPercent = -0.04f;

    //public static final float FFPercent = 0;

    public static final float FFCutoff = -6.296857f;
    
    public static final float maxVelocity = 150;
    public static final float maxAcceleration = 75;

    /* 
    // Constants for smooth initialization
    public static final boolean ENABLE_SMOOTH_INIT = true; // Toggle smooth initialization
    public static final float INIT_SMOOTHING_RATE = 0.025f; // How quickly to approach target (smaller = smoother)
    public static final int INIT_SMOOTHING_TICKS = 100; // Maximum periodic cycles for initialization
    public static final float INIT_COMPLETE_THRESHOLD = 0.5f; // How close to target to consider initialization complete
    */
  }

  public static final class FunnelConstants {
    public static final int ID = 20;
    
    // Position constants
    public static final float homePosition = .555f;
    public static final float fullUpPosition = .24f;
    
    // Angle offset for absolute encoder
    public static final float angleOffset = 0.13f; // Adjust this based on mechanical setup
    
    // Min and max position limits
    public static final float min = .24f;
    public static final float max = .555f;
    
    // Safety threshold - minimum algae arm position that allows funnel movement
    public static final float SAFE_ALGAE_ARM_POSITION = 0.40f;
    
    // Manual control multiplier
    public static final float manualMultiplier = 0.005f;
    
    // Position tolerance for "at position" detection
    public static final float positionTolerance = 0.05f;
    
    // PID Constants
    public static final float P = 1.5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    
    // Motion profile parameters
    public static final float maxVelocity = 50;
    public static final float maxAcceleration = 50;
    public static final float allowedClosedLoopError = 0.005f;
    
    // Smoothing constants (similar to ShooterArm)
    public static final float minSmoothingDistance = 1.0f;  // Minimum angle difference to trigger smoothing
    public static final float approachSmoothingFactor = 0.05f;  // 0-1 value: smaller = smoother/slower approach
    public static final float snapReachThreshold = 0.05f;  // How close funnel must get to snap position before smoothing begins

    // Pre-intake position for coral detection
    public static final float preIntakePosition = 0.52f; // Slightly raised from home position
    
    // Coral detection parameters - reduced threshold and debounce time
    public static final float velocityThreshold = 3.0f; // Lower threshold to detect smaller impacts
    public static final float coralDetectionTime = 0.1f; // Reduced time to confirm detection (seconds)
    
    // Shaking parameters
    public static final float shakingAmplitude = 0.05f; // How far to move when shaking
    public static final float shakingFrequency = 2.5f; // Increased oscillations per second
    public static final float shakingDuration = 10f; // Extended max duration (will stop early when coral loaded)
    
    // Multiple detection methods
    public static final boolean USE_VELOCITY_DETECTION = true;
    public static final boolean USE_CURRENT_DETECTION = true;
  }

  public static final class LEDConstants{
    
    public static final int port = 0;
    public static final int length = 104;

    // LED layout constants
    public static final int LEFT_LEDS = 43;
    public static final int TOP_LEDS = 18;
    public static final int RIGHT_LEDS = 43;
    public static final int TOTAL_LEDS = LEFT_LEDS + TOP_LEDS + RIGHT_LEDS;
    
    // WS2812B LED power specifications
    public static final double MILLIAMPS_PER_LED_FULL_WHITE = 60.0; // mA per LED at full white
    public static final double MILLIAMPS_PER_RED = 20.0;   // mA for full red
    public static final double MILLIAMPS_PER_GREEN = 20.0; // mA for full green
    public static final double MILLIAMPS_PER_BLUE = 20.0;  // mA for full blue
    
    // Power constraint - maximum allowed current in amps
    public static final double MAX_AMPERAGE = 1.0; // Maximum current draw in amps
    
    // Default starting brightness if not using dynamic calculation
    public static final double DEFAULT_BRIGHTNESS = 0.1;
    
    // Power headroom to prevent hitting exact power limit (0-1)
    public static final double POWER_SAFETY_MARGIN = 0.9;
    
    // Calculate max brightness based on power constraints
    // MAX_BRIGHTNESS = MAX_AMPERAGE / (number of LEDs * current per LED at full brightness)
    public static final double MAX_BRIGHTNESS = MAX_AMPERAGE / (length * (MILLIAMPS_PER_LED_FULL_WHITE / 1000.0));  // Convert mA to A

    public static final int ANIMATION_CYCLE_MODULO = 20;
    public static final int FLAME_UPDATE_INTERVAL = 3;
    public static final double NEW_TARGET_PROBABILITY = 0.1;
    public static final double MIN_FLAME_INTENSITY = 0.5;
    public static final double MAX_FLAME_INTENSITY = 1.0;
    public static final double MIN_TARGET_INTENSITY = 0.4;
    public static final double MAX_TARGET_INTENSITY = 1.0;
    public static final double MIN_CHANGE_SPEED = 0.05;
    public static final double MAX_CHANGE_SPEED = 0.2;
    public static final double MIN_BRIGHTNESS_LIMIT = 0.2;
    public static final double MAX_BRIGHTNESS_LIMIT = 1.0;
    
    // Left side flame constants
    public static final double LEFT_SINE_AMPLITUDE = 0.3;
    public static final double LEFT_SINE_OFFSET = 0.7;
    public static final double LEFT_SINE_FREQUENCY = 10.0;
    public static final double LEFT_BOTTOM_INTENSITY = 0.3;
    public static final double LEFT_BOTTOM_OFFSET = 0.7;
    
    // Top flame constants
    public static final double TOP_COSINE_AMPLITUDE = 0.2;
    public static final double TOP_COSINE_OFFSET = 0.8;
    public static final double TOP_COSINE_FREQUENCY = 5.0;
    public static final double TOP_EDGE_INTENSITY = 0.3;
    public static final double TOP_EDGE_OFFSET = 0.7;
    
    // Right side flame constants
    public static final double RIGHT_SINE_AMPLITUDE = 0.3;
    public static final double RIGHT_SINE_OFFSET = 0.7;
    public static final double RIGHT_SINE_FREQUENCY = 10.0;
    public static final double RIGHT_BOTTOM_INTENSITY = 0.3;
    public static final double RIGHT_BOTTOM_OFFSET = 0.7;
    
    // Color variation constants
    public static final double RED_GREEN_MULTIPLIER_BASE = 0.3;
    public static final double RED_GREEN_MULTIPLIER_SCALE = 0.4;
    public static final double BLUE_GREEN_MULTIPLIER = 0.7;
    public static final double BLUE_RED_BRIGHTNESS_MULTIPLIER = 0.1;

    // Breathing pattern constants
    public static final double BREATHING_CYCLE_PERIOD = 1.5; // Full breath cycle in seconds
    public static final double BREATHING_MIN_INTENSITY = 0.05
    ; // Minimum brightness in breath cycle
    public static final double BREATHING_MAX_INTENSITY = 0.8; // Maximum brightness in breath cycle
    public static final int BREATHING_STEPS = 50; // Number of steps per breath (smoother = more steps)

    // Beam pattern constants
    public static final double BEAM_SPEED = 0.8;            // How fast the beams move (higher = faster)
    public static final double BEAM_LENGTH = 0.25;          // Length of beam as fraction of strip segment
    public static final double BEAM_INTENSITY = 0.9;        // Maximum brightness of beam
    public static final double BEAM_TRAIL_LENGTH = 0.5;     // How much the beam fades (higher = longer tail)
    public static final double BEAM_CYCLE_TIME = 2.0;       // Time for full cycle in seconds
    
    // Direction control booleans (true = forward/default, false = reverse)

    public static final boolean LEFT_BEAM_UP = true;        // true = up, false = down
    public static final boolean RIGHT_BEAM_UP = true;       // true = up, false = down
    public static final boolean TOP_BEAM_INWARD = true;     // true = inward, false = outward
  }
  
  public static final class ScopeConstants{
    
  }
  public static final class AprilTagVisionConstants{

    public static final double ambiguityThreshold = 0.1;

    public static final float rightCamXOffset = (float)Units.inchesToMeters(4.814);
    public static final float rightCamYOffset = (float)Units.inchesToMeters(-8.771);
    public static final float rightCamZOffset = (float)Units.inchesToMeters(16.194);

    public static final float rightCamRoll = (float)Units.degreesToRadians(0.0);
    public static final float rightCamPitch = (float)Units.degreesToRadians(15.0);
    public static final float rightCamYaw = (float)Units.degreesToRadians(15.0);

    public static final float leftCamXOffset = (float)Units.inchesToMeters(4.375);
    public static final float leftCamYOffset = (float)Units.inchesToMeters(10.5);
    public static final float leftCamZOffset = (float)Units.inchesToMeters(5);

    public static final float leftCamRoll = (float)Units.degreesToRadians(0.0);
    public static final float leftCamPitch = (float)Units.degreesToRadians(-20.0);
    public static final float leftCamYaw = (float)Units.degreesToRadians(2.5);

    public static final float limelightXOffset = (float)Units.inchesToMeters(-1.588);
    public static final float limelightYOffset = (float)Units.inchesToMeters(2.137);
    public static final float limelightZOffset = (float)Units.inchesToMeters(36.273);

    public static final float limelightRoll = (float)Units.degreesToRadians(0.0);
    public static final float limelightPitch = (float)Units.degreesToRadians(-30.0);
    public static final float limelightYaw = (float)Units.degreesToRadians(180);
    
    // Vision measurement confidence parameters
    public static final double MAX_VISION_DISTANCE_TRUSTED = 0.5;    // Fully trust measurements within this distance (meters)
    public static final double MAX_VISION_DISTANCE_CONSIDERED = 3.0; // Maximum distance to consider a measurement at all (meters)
    public static final double MAX_VISION_ANGLE_TRUSTED = 10.0;      // Fully trust measurements within this angle (degrees)
    public static final double MAX_VISION_ANGLE_CONSIDERED = 30.0;   // Maximum angle diff to consider a measurement (degrees)
    
    // Measurement filtering parameters
    public static final int MIN_CONSECUTIVE_GOOD_MEASUREMENTS = 3;   // Require this many consecutive good measurements
    public static final double OUTLIER_JUMP_DISTANCE = 3.0;          // Maximum plausible movement speed (m/s)
    public static final double OUTLIER_JUMP_ANGLE = 30.0;           // Maximum plausible rotation speed (deg/s)
    
    // Standard deviation parameters for pose estimation
    public static final double BASE_XY_STD_DEV = 0.1;    // Base XY standard deviation (10cm) at max confidence
    public static final double BASE_ROT_STD_DEV = 0.05;  // Base rotation standard deviation (~3 degrees) at max confidence
    
    // Minimum confidence value to prevent division by zero
    public static final double MIN_CONFIDENCE_VALUE = 0.05;
    
    // Initial pose parameters
    public static final double INITIAL_DISTANCE_THRESHOLD = 1.0;     // Distance above which we do full pose reset
    public static final double INITIAL_XY_STD_DEV = 0.1;             // Initial XY standard deviation for close measurements
    public static final double INITIAL_ROT_STD_DEV = 0.05;           // Initial rotation standard deviation for close measurements
  }

  public static final class TargetClassConstants{

    public static final float SLPositionX = (float)Units.inchesToMeters(43.5);
    public static final float SLPositionY = (float)Units.inchesToMeters(278.52);
    public static final float SLPositionZ = (float)Units.degreesToRadians(306);

    public static final float SRPositionX = (float)Units.inchesToMeters(43.5);
    public static final float SRPositionY = (float)Units.inchesToMeters(38.48);
    public static final float SRPositionZ = (float)Units.degreesToRadians(54);
    
    public static final float ProcessorPositionX = (float)Units.inchesToMeters(234);
    public static final float ProcessorPositionY = (float)Units.inchesToMeters(26);
    public static final float ProcessorPositionZ = (float)Units.degreesToRadians(270);

    public static final float xValueC1X0 = (float)Units.inchesToMeters(126);
    public static final float yValueC1X0 = (float)Units.inchesToMeters(163.5);
    public static final float xValueC1X1 = (float)Units.inchesToMeters(126);
    public static final float yValueC1X1 = (float)Units.inchesToMeters(151.13);
    public static final float zValueC1XX = (float)Units.degreesToRadians(0);
    public static final float xValueC130 = (float)Units.inchesToMeters(122.75);
    public static final float yValueC130 = (float)Units.inchesToMeters(163.5);
    public static final float xValueC131 = (float)Units.inchesToMeters(122.75);
    public static final float yValueC131 = (float)Units.inchesToMeters(149.5);
    public static final int faceValueC1XX = 1;

    public static final float xValueC2X0 = (float)Units.inchesToMeters(147.04);
    public static final float yValueC2X0 = (float)Units.inchesToMeters(117.05);
    public static final float xValueC2X1 = (float)Units.inchesToMeters(157.76);
    public static final float yValueC2X1 = (float)Units.inchesToMeters(110.86);
    public static final float zValueC2XX = (float)Units.degreesToRadians(60);
    public static final float xValueC230 = (float)Units.inchesToMeters(145.42);
    public static final float yValueC230 = (float)Units.inchesToMeters(114.24);
    public static final float xValueC231 = (float)Units.inchesToMeters(157.54);
    public static final float yValueC231 = (float)Units.inchesToMeters(107.24);
    public static final int faceValueC2XX = 2;

    
    public static final float xValueC3X0 = (float)Units.inchesToMeters(197.79);
    public static final float yValueC3X0 = (float)Units.inchesToMeters(112.05);
    public static final float xValueC3X1 = (float)Units.inchesToMeters(208.51);
    public static final float yValueC3X1 = (float)Units.inchesToMeters(118.24);
    public static final float zValueC3XX = (float)Units.degreesToRadians(120);
    public static final float xValueC330 = (float)Units.inchesToMeters(199.41);
    public static final float yValueC330 = (float)Units.inchesToMeters(109.23);
    public static final float xValueC331 = (float)Units.inchesToMeters(211.54);
    public static final float yValueC331 = (float)Units.inchesToMeters(116.23);
    public static final int faceValueC3XX = 3;

    
    public static final float xValueC4X0 = (float)Units.inchesToMeters(227.5);
    public static final float yValueC4X0 = (float)Units.inchesToMeters(153.5);
    public static final float xValueC4X1 = (float)Units.inchesToMeters(227.5);
    public static final float yValueC4X1 = (float)Units.inchesToMeters(165.875);
    public static final float zValueC4XX = (float)Units.degreesToRadians(180);
    public static final float xValueC430 = (float)Units.inchesToMeters(230.75);
    public static final float yValueC430 = (float)Units.inchesToMeters(153.5);
    public static final float xValueC431 = (float)Units.inchesToMeters(230.75);
    public static final float yValueC431 = (float)Units.inchesToMeters(167.5);
    public static final int faceValueC4XX = 4;


    
    public static final float xValueC5X0 = (float)Units.inchesToMeters(206.45);
    public static final float yValueC5X0 = (float)Units.inchesToMeters(199.95);
    public static final float xValueC5X1 = (float)Units.inchesToMeters(195.73);
    public static final float yValueC5X1 = (float)Units.inchesToMeters(206.14);
    public static final float zValueC5XX = (float)Units.degreesToRadians(240);
    public static final float xValueC530 = (float)Units.inchesToMeters(208.08);
    public static final float yValueC530 = (float)Units.inchesToMeters(202.77);
    public static final float xValueC531 = (float)Units.inchesToMeters(195.95);
    public static final float yValueC531 = (float)Units.inchesToMeters(209.77);
    public static final int faceValueC5XX = 5;

    
    public static final float xValueC6X0 = (float)Units.inchesToMeters(155.7);
    public static final float yValueC6X0 = (float)Units.inchesToMeters(204.95);
    public static final float xValueC6X1 = (float)Units.inchesToMeters(144.98);
    public static final float yValueC6X1 = (float)Units.inchesToMeters(198.76);
    public static final float zValueC6XX = (float)Units.degreesToRadians(300);
    public static final float xValueC630 = (float)Units.inchesToMeters(154.08);
    public static final float yValueC630 = (float)Units.inchesToMeters(207.77);
    public static final float xValueC631 = (float)Units.inchesToMeters(141.95);
    public static final float yValueC631 = (float)Units.inchesToMeters(200.77);
    public static final int faceValueC6XX = 6;

    public static final int heightCX0X = 0;
    public static final int heightCX1X = 1;
    public static final int heightCX2X = 2;
    public static final int heightCX3X = 3;

    public static final boolean setLeftCXX0 = true;
    public static final boolean setLeftCXX1 = false;
}

  public static final class ZoneConstants {

    public static final float reefCenterX = (float)Units.inchesToMeters(176.745);
    public static final float reefCenterY = (float)Units.inchesToMeters(158.5);
    public static final float reefZoneRadius = (float)Units.inchesToMeters(76);
    
    // Buffer distance to keep outside reef zone until elevator is ready
    public static final float REEF_ZONE_ENTRY_BUFFER = 0.3f;
    
    /*
    // Coral stations (rectangular) - Blue alliance coordinates
    // Right coral station
    public static final float RCoralStationLowerLeftCornerX = (float)Units.inchesToMeters(0);
    public static final float RCoralStationLowerRightCornerX = (float)Units.inchesToMeters(65.83);
    public static final float RCoralStationUpperLeftCornerX = (float)Units.inchesToMeters(21.16);
    public static final float RCoralStationUpperRightCornerX = (float)Units.inchesToMeters(86.99);

    public static final float RCoralStationLowerLeftCornerY = (float)Units.inchesToMeters(47.83);
    public static final float RCoralStationLowerRightCornerY = (float)Units.inchesToMeters(0);
    public static final float RCoralStationUpperLeftCornerY = (float)Units.inchesToMeters(76.95);
    public static final float RCoralStationUpperRightCornerY = (float)Units.inchesToMeters(29.12);

    // Left coral station
    public static final float LCoralStationLowerLeftCornerX = (float)Units.inchesToMeters(0);
    public static final float LCoralStationLowerRightCornerX = (float)Units.inchesToMeters(65.83);
    public static final float LCoralStationUpperLeftCornerX = (float)Units.inchesToMeters(21.16);
    public static final float LCoralStationUpperRightCornerX = (float)Units.inchesToMeters(86.99);

    public static final float LCoralStationLowerLeftCornerY = (float)Units.inchesToMeters(265.08);
    public static final float LCoralStationLowerRightCornerY = (float)Units.inchesToMeters(312.91);
    public static final float LCoralStationUpperLeftCornerY = (float)Units.inchesToMeters(235.95);
    public static final float LCoralStationUpperRightCornerY = (float)Units.inchesToMeters(283.78);
    */
    
    // Adding convenience values for min/max coordinates to make boundary checks easier
    public static final float RCoralStationMinX = (float)Units.inchesToMeters(0);
    public static final float RCoralStationMaxX = (float)Units.inchesToMeters(86.99);
    public static final float RCoralStationMinY = (float)Units.inchesToMeters(0);
    public static final float RCoralStationMaxY = (float)Units.inchesToMeters(76.95);
    
    public static final float LCoralStationMinX = (float)Units.inchesToMeters(0);
    public static final float LCoralStationMaxX = (float)Units.inchesToMeters(86.99);
    public static final float LCoralStationMinY = (float)Units.inchesToMeters(235.95);
    public static final float LCoralStationMaxY = (float)Units.inchesToMeters(312.91);
    
    // Speed multipliers for each zone
    public static final float reefSpeedMultiplier = 0.4f;
    public static final float coralStationMultiplier = 0.4f;


    public static final float speedSmoothingFactor = 0.1f; // Controls how quickly speed changes (0.0-1.0)
  }

  /**
   * Speed control constants for the robot
   */
  public static final class SpeedConstants {
    
    // Default speed when elevator is lowered
    public static final float elevatorLoweredSpeed = 0.8f;
    
    // New speed constants for different elevator heights

    //public static final float elevatorPartiallyRaisedSpeed = 0.5f;

    public static final float elevatorPartiallyRaisedSpeed = 0.4f;

    //public static final float elevatorMidRaisedSpeed = 0.3f;

    public static final float elevatorMidRaisedSpeed = 0.3f;
    public static final float elevatorFullyRaisedSpeed = 0.2f;
    
    // Full speed when at intake position
    public static final float intakePositionSpeed = 0.5f;

    public static float fullSpeedOverride = 1.0f;
  }

  /**
   * Constants for drive-to-pose behavior
   */
  public static final class DriveToPoseConstants {

    // Distance thresholds in meters
    public static final double APPROACHING_DISTANCE_THRESHOLD = 3.5; // meters
    public static final double CLOSE_DISTANCE_THRESHOLD = 1.5; // meters
    public static final double VERY_CLOSE_DISTANCE_THRESHOLD = 0.5; // meters
    
    // Alignment thresholds
    public static final double LINED_UP_ANGLE_THRESHOLD = Math.toRadians(5.0); // radians
    public static final double LINED_UP_POSITION_THRESHOLD = 0.04; // meters
    
    // Visualization settings
    public static final double TARGET_MARKER_SIZE = 0.3; // size of visualization marker

    // Define numerical constraint values for different approach distances
    
    //public static final double APPROACHING_MAX_VEL = 3.0;
    //public static final double APPROACHING_MAX_ACCEL = 2.0;
    
    //public static final double CLOSE_MAX_VEL = 1.5;
    //public static final double CLOSE_MAX_ACCEL = 1.5;
    
    public static final double VERY_CLOSE_MAX_VEL = 6.0;
    public static final double VERY_CLOSE_MAX_ACCEL = 1.5;
    
    // Angular constraints (shared across all distances)
    public static final double MAX_ANGULAR_VEL = Units.degreesToRadians(240);
    public static final double MAX_ANGULAR_ACCEL = Units.degreesToRadians(120);
  }
}