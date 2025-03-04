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

  public static final float DEADBAND = 0.1f;

  public static final float armLength = (float)Units.inchesToMeters(0);

  public static final float closeToPoseErrorAllowance = 0.05f;
  


  
  public static final class AlgaeArmConstants{
    public static final int ID = 14;

    public static final float stowedUpAngle = 0.28f;
    public static final float straightOutAngle = 0.5f;
    public static final float groundIntakeAngle = 0.406f;

    public static final float holdAngle = 0.35f;

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
    public static final float intake = 0.3f;
    public static final float outake = -0.4f;
    public static final float currentThreshold = 15.0f; // Current threshold for detecting algae
    
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

    public static final float angleOffset = .5f;

    public static final float manualMultiplier = -0.005f;

    public static final float min = 0.225f;
    //public static final float max = .9f;
    public static final float max = 0.698f;

    public static final float maxManual = 0.698f;
    //public static final float maxManual = .9f;
    
    public static final float climbAngle = 0.698f; //Larger than this in reality

    public static final float scoreAngleLOW = 0.698f;
    public static final float scoreAngleHIGH = 0.643f;
    public static final float loadAngle = 0.225f;
    
    // Ball position angle
    public static final float ballAngle = 0.5f;

    public static final float L1Angle = 0.5f;

    public static final float P = 3.5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 1000;
    public static final float maxAcceleration = 200;
    public static final float allowedClosedLoopError = .05f;
  }

  public static final class ShooterPivotConstants{

    public static final int ID = 17;

    public static final float leftAngleInitial = 0.382339f;
    public static final float rightAngleInitial = 0.622852f;
    public static final float centerAngle = .503f;

    //Pivot Set Positions
    public static final float leftL2Angle = 0.336f;
    public static final float rightL2Angle = 0.65666f;
    public static final float leftL3Angle = 0.281f;
    public static final float rightL3Angle = 0.64f;
    public static final float leftL4Angle = 0.392f;
    public static final float rightL4Angle = 0.688f;

    public static final float angleOffset = 0.48f;

    public static final float max = 0.73f;
    public static final float min = 0.255f;

    public static final float manualMultiplier = 0.005f;

    public static final float P = 5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 250;
    public static final float maxAcceleration = 100;
    public static final float allowedClosedLoopError = .05f;
  }

  public static final class ClimberConstants{

    public static final int ID = 18;

    public static final float extendSpeed = 1;
    public static final float retractSpeed = -1f;
  }
  
  public static final class ElevatorConstants{
    
    public static final int ID = 13;
    
    public static final float manualMultiplier = 2f;
    //public static final float manualMultiplier = .2f; //For Testing

    //Old with Orange Spools
    /*
    public static final float min = -163.2f;
    public static final float max = 0.0f;

    public static final float L4Pose = -163.2f;
    public static final float L3Pose = -94.6625f;
    public static final float L2Pose = -41.4f;
    public static final float L1Pose = 0.0f;
    public static final float pickupPose = -75f;
    */
    //Gray Spools
    public static final float min = -237f;
    public static final float max = 0.0f;

    // elevator set positions
    public static final float L4Pose = -237f;
    public static final float L3LPose = -133f;
    public static final float L3RPose = -123f;
    public static final float L2RPose = -50f;
    public static final float L2LPose = -43f;
    public static final float L1Pose = -44f;
    public static final float pickupPose = -104;
    public static final float climbPose = -122.333374f;
    
    // Ball pickup positions
    public static final float highBallPose = -150f;
    public static final float lowBallPose = -75f;
    
    public static final float P = 0.25f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 4500;
    public static final float maxAcceleration = 6000;
    public static final float allowedClosedLoopError = .5f;
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
    public static final float rightCamYOffset = (float)Units.inchesToMeters(-8.021);
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
    public static final float SLPositionY = (float)Units.inchesToMeters(274.43);
    public static final float SLPositionZ = (float)Units.degreesToRadians(226);

    public static final float SRPositionX = (float)Units.inchesToMeters(43.5);
    public static final float SRPositionY = (float)Units.inchesToMeters(38.48);
    public static final float SRPositionZ = (float)Units.degreesToRadians(54);
    
    public static final float ProcessorPositionX = (float)Units.inchesToMeters(234);
    public static final float ProcessorPositionY = (float)Units.inchesToMeters(26);
    public static final float ProcessorPositionZ = (float)Units.degreesToRadians(270);

    public static final float xValueC1XX = (float)Units.inchesToMeters(126);
    public static final float yValueC1XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC1XX = (float)Units.degreesToRadians(0);
    public static final float xValueC130 = (float)Units.inchesToMeters(122.5);
    public static final float yValueC130 = (float)Units.inchesToMeters(166.5);
    public static final float xValueC131 = (float)Units.inchesToMeters(123.5);
    public static final float yValueC131 = (float)Units.inchesToMeters(153);
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

    public static final float xValueC2XX = (float)Units.inchesToMeters(151.37);
    public static final float yValueC2XX = (float)Units.inchesToMeters(114.55);
    public static final float zValueC2XX = (float)Units.degreesToRadians(60);
    public static final float xValueC230 = (float)Units.inchesToMeters(142.69);
    public static final float yValueC230 = (float)Units.inchesToMeters(115.52);
    public static final float xValueC231 = (float)Units.inchesToMeters(154.88);
    public static final float yValueC231 = (float)Units.inchesToMeters(109.63);
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

    
    public static final float xValueC3XX = (float)Units.inchesToMeters(202.12);
    public static final float yValueC3XX = (float)Units.inchesToMeters(114.55);
    public static final float zValueC3XX = (float)Units.degreesToRadians(120);
    public static final float xValueC330 = (float)Units.inchesToMeters(196.94);
    public static final float yValueC330 = (float)Units.inchesToMeters(107.52);
    public static final float xValueC331 = (float)Units.inchesToMeters(210.66);
    public static final float yValueC331 = (float)Units.inchesToMeters(113.55);
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

    
    public static final float xValueC4XX = (float)Units.inchesToMeters(227.5);
    public static final float yValueC4XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC4XX = (float)Units.degreesToRadians(180);
    public static final float xValueC430 = (float)Units.inchesToMeters(231);
    public static final float yValueC430 = (float)Units.inchesToMeters(150.5);
    public static final float xValueC431 = (float)Units.inchesToMeters(230);
    public static final float yValueC431 = (float)Units.inchesToMeters(164);
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


    
    public static final float xValueC5XX = (float)Units.inchesToMeters(202.12);
    public static final float yValueC5XX = (float)Units.inchesToMeters(202.45);
    public static final float zValueC5XX = (float)Units.degreesToRadians(240);
    public static final float xValueC530 = (float)Units.inchesToMeters(210.8);
    public static final float yValueC530 = (float)Units.inchesToMeters(201.48);
    public static final float xValueC531 = (float)Units.inchesToMeters(198.61);
    public static final float yValueC531 = (float)Units.inchesToMeters(207.37);
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

    
    public static final float xValueC6XX = (float)Units.inchesToMeters(151.37);
    public static final float yValueC6XX = (float)Units.inchesToMeters(202.45);
    public static final float zValueC6XX = (float)Units.degreesToRadians(300);
    public static final float xValueC630 = (float)Units.inchesToMeters(156.55);
    public static final float yValueC630 = (float)Units.inchesToMeters(209.48);
    public static final float xValueC631 = (float)Units.inchesToMeters(145.36);
    public static final float yValueC631 = (float)Units.inchesToMeters(201.87);
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

    public static final float reefCenterX = 8.27f; // Center of the reef in meters
    public static final float reefCenterY = 4.11f; // Center of the reef in meters  
    public static final float reefZoneRadius = 1.8f;    // Radius of the reef in meters
    
    // Coral stations (rectangular) - Blue alliance coordinates
    // Left coral station
    public static final float LCoralStationMinX = (float)Units.inchesToMeters(0);
    public static final float LCoralStationMaxX = (float)Units.inchesToMeters(0);
    public static final float LCoralStationMinY = (float)Units.inchesToMeters(0);
    public static final float LCoralStationMaxY = (float)Units.inchesToMeters(0);
    
    // Right coral station
    public static final float RCoralStationMinX = (float)Units.inchesToMeters(0);
    public static final float RCoralStationMaxX = (float)Units.inchesToMeters(0);
    public static final float RCoralStationMinY = (float)Units.inchesToMeters(0);
    public static final float RCoralStationMaxY = (float)Units.inchesToMeters(0);
    
    // Speed multipliers for each zone
    public static final float reefSpeedMultiplier = 0.4f;
    public static final float coralStationMultiplier = 0.6f;


    public static final float speedSmoothingFactor = 0.05f; // Controls how quickly speed changes (0.0-1.0)
  }

  /**
   * Speed control constants for the robot
   */
  public static final class SpeedConstants {
    // Default speed when elevator is raised
    public static final float elevatorRaisedSpeed = 0.3f;
    
    // Default speed when elevator is lowered
    public static final float elevatorLoweredSpeed = 0.5f;
    
    // Speed reduction factors for various distances to target
    public static final float veryCloseSpeedFactor = 0.5f;
    public static final float closeSpeedFactor = 0.5f;
    public static final float approachingSpeedFactor = 0.5f;
    
    // Distance thresholds for various proximity classifications
    public static final float veryCloseDistance = 0.5f;
    public static final float closeDistance = 1.0f;
    public static final float approachingDistance = 1.5f;
    public static final float linedUpDistance = 0.08f;
  }
}