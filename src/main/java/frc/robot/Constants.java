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




  
  public static final class AlgaeArmConstants{
    public static final int ID = 22;

    public static final float stowedUpAngle = 0.0f;
    public static final float straightOutAngle = 0.0f;
    public static final float groundIntakeAngle = 0.0f;

    public static final float angleOffset = 0.0f;

    public static final float max = 0.0f;
    public static final float min = 0.0f;

    public static final float manualMultiplier = 0.1f;


    public static final float P = 0.0f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 2100;
    public static final float maxAcceleration = 3000;
    public static final float allowedClosedLoopError = .5f;
  }

  public static final class AlgaeShooterConstants{
    public static final int ID = 10;
    public static final float intake = -0.1f;
    public static final float outake = 0.1f;
  }
  public static final class ShooterConstants{
    public static final int ID = 9;
    public static final float intake = 0.15f;
    public static final float outake = -0.15f;
  }

  public static final class ShooterArmConstants{
    public static final int ID = 21;

    public static final float angleOffset = 0;

    public static final float manualMultiplier = 0.1f;

    public static final float min = 0.0f;
    public static final float max = 0.0f;

    public static final float scoreAngle = 0.0f;
    public static final float loadAngle = 0.0f;
    public static final float L1Angle = 0.0f;

    public static final float P = 0.0f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 2100;
    public static final float maxAcceleration = 3000;
    public static final float allowedClosedLoopError = .5f;
  }

  public static final class ShooterPivotConstants{

    public static final int ID = 11;

    public static final float leftAngleInitial = 0.0f;
    public static final float rightAngleInitial = 0.0f;
    public static final float centerAngle = 0;

    public static final float angleOffset = 0.0f;

    public static final float max = 0.0f;
    public static final float min = 0.0f;

    public static final float manualMultiplier = 0.1f;

    public static final float P = 0.0f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 2100;
    public static final float maxAcceleration = 3000;
    public static final float allowedClosedLoopError = .5f;
  }

  public static final class AimingConstants{

  }

  public static final class ClimberConstants{

    public static final int ID = 20;

    public static final float extendSpeed = 0.2f;
    public static final float retractSpeed = -0.2f;
  }
  
  public static final class ElevatorConstants{

    public static final int ID = 0;

    public static final float maxSpeed = 0.1f;
    
    public static final float manualMultiplier = 0.1f;

    public static final float min = 0.0f;
    public static final float max = 0.0f;

    public static final float L4Pose = 0.0f;
    public static final float L3Pose = 0.0f;
    public static final float L2Pose = 0.0f;
    public static final float L1Pose = 0.0f;


    public static final float P = 0.0f;
    public static final float I = 0.0f;
    public static final float D = 0.0f;
    public static final float maxVelocity = 2100;
    public static final float maxAcceleration = 3000;
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

    public static final float rightCamXOffset = 0.0f;
    public static final float rightCamYOffset = 0.0f;
    public static final float rightCamZOffset = 0.0f;

    public static final float rightCamRoll = 0.0f;
    public static final float rightCamPitch = 0.0f;
    public static final float rightCamYaw = 0.0f;

    
    public static final float leftCamXOffset = 0.0f;
    public static final float leftCamYOffset = 0.0f;
    public static final float leftCamZOffset = 0.0f;

    public static final float leftCamRoll = 0.0f;
    public static final float leftCamPitch = 0.0f;
    public static final float leftCamYaw = 0.0f;

    public static final float limelightXOffset = 0.0f;
    public static final float limelightYOffset = 0.0f;
    public static final float limelightZOffset = 0.0f;

    public static final float limelightRoll = 0.0f;
    public static final float limelightPitch = 0.0f;
    public static final float limelightYaw = 0.0f;
  }

  public static final class AlignmentConstants{

    public static final float P = 0.05f;
    public static final float maxSpeed = 0.5f;
    //Scoring Poses Below

  }

}