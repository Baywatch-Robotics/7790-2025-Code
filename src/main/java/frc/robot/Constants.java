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

    public static final float stowedUpAngle = 0.28f;
    public static final float straightOutAngle = 0.5f;
    public static final float groundIntakeAngle = 0.42f;

    public static final float holdAngle = 0.34f;

    // Current threshold for detecting algae
    public static final float currentThreshold = 15.0f;

    public static final float angleOffset = 0.44f;

    public static final float max = 0.5f;
    public static final float min = 0.28f;

    public static final float manualMultiplier = 0.05f;


    public static final float P = 2f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 1000;
    public static final float maxAcceleration = 1000;
    public static final float allowedClosedLoopError = .05f;
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
  }
  public static final class ShooterConstants{
    public static final int ID = 19;
    public static final float intake = 0.15f;
    public static final float outake = -.50f;
    public static final float currentThreshold = 15;
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
    public static final float scoreAngleHIGH = 0.66f;
    // Minimum angle allowed in reef zone (prevents arm from going too low in reef)
    public static final float reefZoneMinimumAngle = 0.643f; // This should be between ballAngle and scoreAngleHIGH
    // Debounce time when exiting reef zone (seconds)
    public static final float reefZoneExitDebounceTime = 0.5f;

    public static final float loadAngle = 0.252f
    ;
    
    // Ball position angle
    public static final float ballAngle = 0.5f;

    public static final float L1Angle = 0.5f;

    public static final float P = 5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    //public static final float maxVelocity = 1000;
    //public static final float maxAcceleration = 200;

    public static final float maxVelocity = 100f;
    public static final float maxAcceleration = 100f;


    public static final float allowedClosedLoopError = .05f;

    public static final float minSmoothingDistance = 1.0f;  // Minimum angle difference to trigger smoothing
    public static final float approachSmoothingFactor = 0.05f;  // 0-1 value: smaller = smoother/slower approach
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
    public static final float min = -46.767197f;
    public static final float max = 0.0f;

    // elevator set positions
    public static final float L4Pose = -46.767197f;
    public static final float L3LPose = -22.977661f;
    public static final float L3RPose = -22.977661f
    ;
    public static final float L2RPose = -8.41037f;
    public static final float L2LPose = -8.41037f;
    public static final float L1Pose = -0;
    public static final float pickupPose = -22f;
    public static final float climbPose = -0;

    //-6.296857f WHEN TO DISABLE FEED FORWARD
    
    public static final float downPosition = -7f;
    
    // Ball pickup positions
    // public static final float highBallPose = -50f;
    // public static final float lowBallPose = -25f;
    
    // New elevator height thresholds for speed control
    public static final float SLIGHTLY_RAISED_THRESHOLD = -10f;
    public static final float PARTIALLY_RAISED_THRESHOLD = -20f;
    public static final float MID_RAISED_THRESHOLD = -30f;
    public static final float FULLY_RAISED_THRESHOLD = -40f;
    
    // Elevator setpoint tolerance constants
    public static final float STANDARD_SETPOINT_TOLERANCE = 2.0f;
    public static final float NEAR_SETPOINT_TOLERANCE = 5.0f;
    public static final float APPROACHING_SETPOINT_TOLERANCE = 7.5f;
    
    // Home position threshold
    public static final float HOME_POSITION_THRESHOLD = 5.0f;
    
    // Intake clearance thresholds
    public static final float INTAKE_CLEARANCE_MARGIN = 5.0f;
    
    public static final float P = .25f;
    public static final float I = 0.0f;
    public static final float D = 0f;

    public static final float FFPercent = -0.04f;

    public static final float FFCutoff = -6.296857f;
    
    public static final float maxVelocity = 2750;
    public static final float maxAcceleration = 6000;
    public static final float allowedClosedLoopError = .1f;
  }

  public static final class FunnelConstants{
    
  }

  public static final class LEDConstants{
    
    public static final int port = 0;

    public static final int length = 25;

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
    public static final float yValueC1X1 = (float)Units.inchesToMeters(151.125);
    public static final float zValueC1XX = (float)Units.degreesToRadians(0);
    public static final float xValueC130 = (float)Units.inchesToMeters(126);
    public static final float yValueC130 = (float)Units.inchesToMeters(163.5);
    public static final float xValueC131 = (float)Units.inchesToMeters(126);
    public static final float yValueC131 = (float)Units.inchesToMeters(151.125);
    /*
    public static final float xValueC1XX = (float)Units.inchesToMeters(126);
    public static final float yValueC1XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC1XX = (float)Units.degreesToRadians(0);
    public static final float xValueC130 = (float)Units.inchesToMeters(121);
    public static final float yValueC130 = (float)Units.inchesToMeters(165.5);
    public static final float xValueC131 = (float)Units.inchesToMeters(121);
    public static final float yValueC131 = (float)Units.inchesToMeters(151.5);
    */
    public static final int faceValueC1XX = 1;

    public static final float xValueC2X0 = (float)Units.inchesToMeters(147.04);
    public static final float yValueC2X0 = (float)Units.inchesToMeters(117.05);
    public static final float xValueC2X1 = (float)Units.inchesToMeters(157.76);
    public static final float yValueC2X1 = (float)Units.inchesToMeters(110.86);
    public static final float zValueC2XX = (float)Units.degreesToRadians(60);
    public static final float xValueC230 = (float)Units.inchesToMeters(147.04);
    public static final float yValueC230 = (float)Units.inchesToMeters(117.05);
    public static final float xValueC231 = (float)Units.inchesToMeters(157.76);
    public static final float yValueC231 = (float)Units.inchesToMeters(110.86);
    /*
    public static final float xValueC2XX = (float)Units.inchesToMeters(151.39);
    public static final float yValueC2XX = (float)Units.inchesToMeters(114.58);
    public static final float zValueC2XX = (float)Units.degreesToRadians(60);
    public static final float xValueC230 = (float)Units.inchesToMeters(142.83);
    public static final float yValueC230 = (float)Units.inchesToMeters(113.55);
    public static final float xValueC231 = (float)Units.inchesToMeters(154.95);
    public static final float yValueC231 = (float)Units.inchesToMeters(106.75);
    */
    public static final int faceValueC2XX = 2;

    
    public static final float xValueC3X0 = (float)Units.inchesToMeters(197.79);
    public static final float yValueC3X0 = (float)Units.inchesToMeters(112.05);
    public static final float xValueC3X1 = (float)Units.inchesToMeters(208.51);
    public static final float yValueC3X1 = (float)Units.inchesToMeters(118.24);
    public static final float zValueC3XX = (float)Units.degreesToRadians(120);
    public static final float xValueC330 = (float)Units.inchesToMeters(197.79);
    public static final float yValueC330 = (float)Units.inchesToMeters(112.05);
    public static final float xValueC331 = (float)Units.inchesToMeters(208.51);
    public static final float yValueC331 = (float)Units.inchesToMeters(118.24);
    /*
    public static final float xValueC3XX = (float)Units.inchesToMeters(202.1);
    public static final float yValueC3XX = (float)Units.inchesToMeters(114.58);
    public static final float zValueC3XX = (float)Units.degreesToRadians(120);
    public static final float xValueC330 = (float)Units.inchesToMeters(198.54);
    public static final float yValueC330 = (float)Units.inchesToMeters(106.75);
    public static final float xValueC331 = (float)Units.inchesToMeters(210.66);
    public static final float yValueC331 = (float)Units.inchesToMeters(113.55);
    */
    public static final int faceValueC3XX = 3;

    
    public static final float xValueC4X0 = (float)Units.inchesToMeters(227.5);
    public static final float yValueC4X0 = (float)Units.inchesToMeters(153.5);
    public static final float xValueC4X1 = (float)Units.inchesToMeters(227.5);
    public static final float yValueC4X1 = (float)Units.inchesToMeters(165.875);
    public static final float zValueC4XX = (float)Units.degreesToRadians(180);
    public static final float xValueC430 = (float)Units.inchesToMeters(231);
    public static final float yValueC430 = (float)Units.inchesToMeters(153.5);
    public static final float xValueC431 = (float)Units.inchesToMeters(230);
    public static final float yValueC431 = (float)Units.inchesToMeters(165.875);
    /*
    public static final float xValueC4XX = (float)Units.inchesToMeters(227.49);
    public static final float yValueC4XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC4XX = (float)Units.degreesToRadians(180);
    public static final float xValueC430 = (float)Units.inchesToMeters(232.49);
    public static final float yValueC430 = (float)Units.inchesToMeters(151.5);
    public static final float xValueC431 = (float)Units.inchesToMeters(232.49);
    public static final float yValueC431 = (float)Units.inchesToMeters(165.5);
    */
    public static final int faceValueC4XX = 4;


    
    public static final float xValueC5X0 = (float)Units.inchesToMeters(206.45);
    public static final float yValueC5X0 = (float)Units.inchesToMeters(199.95);
    public static final float xValueC5X1 = (float)Units.inchesToMeters(195.73);
    public static final float yValueC5X1 = (float)Units.inchesToMeters(206.14);
    public static final float zValueC5XX = (float)Units.degreesToRadians(240);
    public static final float xValueC530 = (float)Units.inchesToMeters(206.45);
    public static final float yValueC530 = (float)Units.inchesToMeters(199.95);
    public static final float xValueC531 = (float)Units.inchesToMeters(195.73);
    public static final float yValueC531 = (float)Units.inchesToMeters(206.14);
    /*
    public static final float xValueC5XX = (float)Units.inchesToMeters(202.1);
    public static final float yValueC5XX = (float)Units.inchesToMeters(202.42);
    public static final float zValueC5XX = (float)Units.degreesToRadians(240);
    public static final float xValueC530 = (float)Units.inchesToMeters(210.66);
    public static final float yValueC530 = (float)Units.inchesToMeters(203.25);
    public static final float xValueC531 = (float)Units.inchesToMeters(198.54);
    public static final float yValueC531 = (float)Units.inchesToMeters(210.98);
    */
    public static final int faceValueC5XX = 5;

    
    public static final float xValueC6X0 = (float)Units.inchesToMeters(155.7);
    public static final float yValueC6X0 = (float)Units.inchesToMeters(204.95);
    public static final float xValueC6X1 = (float)Units.inchesToMeters(144.98);
    public static final float yValueC6X1 = (float)Units.inchesToMeters(198.76);
    public static final float zValueC6XX = (float)Units.degreesToRadians(300);
    public static final float xValueC630 = (float)Units.inchesToMeters(155.7);
    public static final float yValueC630 = (float)Units.inchesToMeters(204.95);
    public static final float xValueC631 = (float)Units.inchesToMeters(144.98);
    public static final float yValueC631 = (float)Units.inchesToMeters(198.76);
    /*
    public static final float xValueC6XX = (float)Units.inchesToMeters(151.39);
    public static final float yValueC6XX = (float)Units.inchesToMeters(202.42);
    public static final float zValueC6XX = (float)Units.degreesToRadians(300);
    public static final float xValueC630 = (float)Units.inchesToMeters(154.95);
    public static final float yValueC630 = (float)Units.inchesToMeters(210.98);
    public static final float xValueC631 = (float)Units.inchesToMeters(142.83);
    public static final float yValueC631 = (float)Units.inchesToMeters(203.25);
    */
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
    public static final double APPROACHING_DISTANCE_THRESHOLD = 1.5; // meters
    public static final double CLOSE_DISTANCE_THRESHOLD = 1.0; // meters
    public static final double VERY_CLOSE_DISTANCE_THRESHOLD = 0.3; // meters
    
    // Alignment thresholds
    public static final double LINED_UP_ANGLE_THRESHOLD = Math.toRadians(5.0); // radians
    public static final double LINED_UP_POSITION_THRESHOLD = 0.02; // meters
    
    // Visualization settings
    public static final double TARGET_MARKER_SIZE = 0.3; // size of visualization marker

    // Define numerical constraint values for different approach distances
    public static final double APPROACHING_MAX_VEL = 4.5;
    public static final double APPROACHING_MAX_ACCEL = 3.0;
    
    public static final double CLOSE_MAX_VEL = 2.0;
    public static final double CLOSE_MAX_ACCEL = 1.0;
    
    public static final double VERY_CLOSE_MAX_VEL = 1.0;
    public static final double VERY_CLOSE_MAX_ACCEL = 1.0;
    
    // Angular constraints (shared across all distances)
    public static final double MAX_ANGULAR_VEL = Units.degreesToRadians(240);
    public static final double MAX_ANGULAR_ACCEL = Units.degreesToRadians(120);
  }
}