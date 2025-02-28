// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    public static final float stowedUpAngle = 0.3f;
    public static final float straightOutAngle = 0.5f;
    public static final float groundIntakeAngle = 0.5f;

    public static final float angleOffset = 0.44f;

    public static final float max = 0.5f;
    public static final float min = 0.28f;

    public static final float manualMultiplier = 0.05f;


    public static final float P = 1.5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 1000;
    public static final float maxAcceleration = 1000;
    public static final float allowedClosedLoopError = .05f;
  }

    public static final class AlgaeShooterConstants{
    public static final int ID = 16;
    public static final float intake = -0.3f;
    public static final float outake = 0.3f;
  }
  public static final class ShooterConstants{
    public static final int ID = 19;
    public static final float intake = 0.15f;
    public static final float outake = -1f;
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
    public static final float scoreAngleHIGH = 0.68f;
    public static final float loadAngle = 0.225f;

    // L4 Arm Angles
    public static final float LeftL4Angle = .669f;
    public static final float RightL4Angle = .689f;

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
    public static final float centerAngle = .49f;

    //Pivot Set Positions
    public static final float leftL2Angle = 0.336f;
    public static final float rightL2Angle = 0.65666f;
    public static final float leftL3Angle = 0.336f;
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

    public static final float extendSpeed = .2f;
    public static final float retractSpeed = -.2f;
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
    public static final float L3LPose = -122.16f;
    public static final float L3RPose = -121.8f;
    public static final float L2RPose = -47.85f;
    public static final float L2LPose = -40f;
    public static final float L1Pose = -41f;
    public static final float pickupPose = -92.3125f;
    public static final float climbPose = -122.333374f;
    
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
    public static final float leftCamPitch = (float)Units.degreesToRadians(20.0);
    public static final float leftCamYaw = (float)Units.degreesToRadians(-20.0);

    public static final float limelightXOffset = (float)Units.inchesToMeters(-1.588);
    public static final float limelightYOffset = (float)Units.inchesToMeters(2.137);
    public static final float limelightZOffset = (float)Units.inchesToMeters(36.273);

    public static final float limelightRoll = (float)Units.degreesToRadians(0.0);
    public static final float limelightPitch = (float)Units.degreesToRadians(-30.0);
    public static final float limelightYaw = (float)Units.degreesToRadians(180);
  }

  public static final class TargetClassConstants{

    public static final float SLPositionX = (float)Units.inchesToMeters(44.09);
    public static final float SLPositionY = (float)Units.inchesToMeters(276.64);
    public static final float SLPositionZ = (float)Units.degreesToRadians(306-180);

    public static final float SRPositionX = (float)Units.inchesToMeters(44.09);
    public static final float SRPositionY = (float)Units.inchesToMeters(40.36);
    public static final float SRPositionZ = (float)Units.degreesToRadians(54+180);
    
    public static final float ProcessorPositionX = (float)Units.inchesToMeters(234);
    public static final float ProcessorPositionY = (float)Units.inchesToMeters(26);
    public static final float ProcessorPositionZ = (float)Units.degreesToRadians(270);

    public static final float xValueC1XX = (float)Units.inchesToMeters(126);
    public static final float yValueC1XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC1XX = (float)Units.degreesToRadians(0);
    public static final int faceValueC1XX = 1;

    public static final float xValueC2XX = (float)Units.inchesToMeters(151.39);
    public static final float yValueC2XX = (float)Units.inchesToMeters(114.58);
    public static final float zValueC2XX = (float)Units.degreesToRadians(240-180);
    public static final int faceValueC2XX = 2;


    public static final float xValueC3XX = (float)Units.inchesToMeters(202.1);
    public static final float yValueC3XX = (float)Units.inchesToMeters(114.58);
    public static final float zValueC3XX = (float)Units.degreesToRadians(300-180);
    public static final int faceValueC3XX = 3;

    public static final float xValueC4XX = (float)Units.inchesToMeters(227.49);
    public static final float yValueC4XX = (float)Units.inchesToMeters(158.5);
    public static final float zValueC4XX = (float)Units.degreesToRadians(180);
    public static final int faceValueC4XX = 4;

    public static final float xValueC5XX = (float)Units.inchesToMeters(202.1);
    public static final float yValueC5XX = (float)Units.inchesToMeters(202.42);
    public static final float zValueC5XX = (float)Units.degreesToRadians(60+180);
    public static final int faceValueC5XX = 5;

    public static final float xValueC6XX = (float)Units.inchesToMeters(151.39);
    public static final float yValueC6XX = (float)Units.inchesToMeters(202.42);
    public static final float zValueC6XX = (float)Units.degreesToRadians(120+180);
    public static final int faceValueC6XX = 6;

    public static final int heightCX0X = 0;
    public static final int heightCX1X = 1;
    public static final int heightCX2X = 2;
    public static final int heightCX3X = 3;

    public static final boolean setLeftCXX0 = true;
    public static final boolean setLeftCXX1 = false;
}
}