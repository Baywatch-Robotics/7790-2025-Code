// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static final float closeToPoseErrorAllowance = 0.03f;
  


  
  public static final class AlgaeArmConstants{
    public static final int ID = 14;

    public static final float stowedUpAngle = 0.74f;
    public static final float straightOutAngle = 0.5f;
    public static final float groundIntakeAngle = 0.5f; //Tenative could move arm up and then angle farther down
    public static final float upperCrashLimit = 0.75f; // This is absolute max, there are two sub states where the max is either equal to this or slightly less due to the placement mechanism being in the way
    public static final float lowerCrashLimit = 0.45f; // Slightly ahead of actually limit for safety
    public static final float clearanceAngle = 0.5f;

    public static final float angleOffset = 0.56f;

    public static final float max = 0.5f;
    public static final float min = 0.31f;

    public static final float manualMultiplier = 0.005f;


    public static final float P = 0.5f;
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

    public static final float manualMultiplier = 0.005f;

    public static final float min = 0.225f;
    //public static final float max = .9f;
    public static final float max = 0.698f;

    public static final float maxManual = 0.698f;
    //public static final float maxManual = .9f;
    
    public static final float climbAngle = 0.698f; //Larger than this in reality

    public static final float scoreAngleLOW = 0.698f;
    public static final float scoreAngleHIGH = 0.68f;
    public static final float loadAngle = 0.225f;
    public static final float L1Angle = 0.5f;

    public static final float P = 2f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 1000;
    public static final float maxAcceleration = 250;
    public static final float allowedClosedLoopError = .05f;
  }

  public static final class ShooterPivotConstants{

    public static final int ID = 17;

    public static final float leftAngleInitial = 0.382339f;
    public static final float rightAngleInitial = 0.622852f;
    public static final float centerAngle = .49f;

    public static final float angleOffset = 0.48f;

    public static final float max = 0.73f;
    public static final float min = 0.255f;

    public static final float manualMultiplier = 0.005f;

    public static final float P = 2.5f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 1000;
    public static final float maxAcceleration = 250;
    public static final float allowedClosedLoopError = .05f;
  }

  public static final class ClimberConstants{

    public static final int ID = 18;

    public static final float extendSpeed = 1f;
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

    public static final float L4Pose = -237f;
    public static final float L3Pose = -116.152504f;
    public static final float L2Pose = -31.551039f;
    public static final float L1Pose = -41f;
    public static final float pickupPose = -95f;
    public static final float climbPose = -116;
    
    public static final float P = 0.2f;
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

    
    public static final float leftCamXOffset = (float)Units.inchesToMeters(3.432);
    public static final float leftCamYOffset = (float)Units.inchesToMeters(9.562);
    public static final float leftCamZOffset = (float)Units.inchesToMeters(23.941);

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

    public static final float xValueC0XX = 5.795f;
    public static final float yValueC0XX = 4.0f;
    public static final float zValueC0XX = (float)Units.degreesToRadians(180.0f);
    public static final int faceValueC0XX = 0;
    
    public static final float xValueC1XX = 5.144f;
    public static final float yValueC1XX = 5.160f;
    public static final float zValueC1XX = (float)Units.degreesToRadians(-120.0f);
    public static final int faceValueC1XX = 1;

    public static final float xValueC2XX = 3.832f;
    public static final float yValueC2XX = 5.156f;
    public static final float zValueC2XX = (float)Units.degreesToRadians(-60.0f);
    public static final int faceValueC2XX = 2;


    public static final float xValueC3XX = 3.185f; //USE

    //public static final float xValueC3XX = 2.750f;
    public static final float yValueC3XX = 4.03f;
    public static final float zValueC3XX = (float)Units.degreesToRadians(0.0f);;
    public static final int faceValueC3XX = 3;

    public static final float xValueC4XX = 3.832f;
    public static final float yValueC4XX = 2.892f;
    public static final float zValueC4XX = (float)Units.degreesToRadians(60.0f);
    public static final int faceValueC4XX = 4;

    public static final float xValueC5XX = 5.134f;
    public static final float yValueC5XX = 2.893f;
    public static final float zValueC5XX = (float)Units.degreesToRadians(120.0f);
    public static final int faceValueC5XX = 5;

    public static final int heightCX0X = 0;
    public static final int heightCX1X = 1;
    public static final int heightCX2X = 2;
    public static final int heightCX3X = 3;

    public static final boolean setLeftCXX0 = true;
    public static final boolean setLeftCXX1 = false;
}

public static final class DriveToPoseConstants{

    public static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(1, 1);
    public static final ProfiledPIDController xProfiledPID = new ProfiledPIDController(.5, 0, 0, xConstraints);

    public static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(1, 1);
    public static final ProfiledPIDController yProfiledPID = new ProfiledPIDController(.5, 0, 0, yConstraints);
}
}