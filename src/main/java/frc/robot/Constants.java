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

  public static final float ROBOT_MASS = (float) Units.lbsToKilograms(110); 
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final float LOOP_TIME  = 0.13f; //s, 20ms + 110ms sprk max velocity lag
  public static final float MAX_SPEED  = (float) Units.feetToMeters(22.1);
  // Maximum speed of the robot in meters per second, used to limit acceleration.




//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final float WHEEL_LOCK_TIME = 10f; // seconds
  }

  public static final class OperatorConstants
  {

    // Joystick Deadband
    public static final float DEADBAND        = 0.1f;
    public static final float LEFT_Y_DEADBAND = 0.1f;
    public static final float RIGHT_X_DEADBAND = 0.1f;
    public static final float TURN_CONSTANT    = 6f;
  }



  
  public static final class BallArmConstants{
    
  }

  public static final class ClimberConstants{
    
  }
  
  public static final class ElevatorConstants{

    public static final int elevatorID = 0;

    public static final float elevatorMaxSpeed = 0.1f;
    
    public static final float minimumExtension = 0.0f;
    public static final float maximumExtension = 0.0f;


    public static final float elevatorP = 0.0f;
    public static final float elevatorI = 0.0f;
    public static final float elevatorD = 0.0f;
    public static final float elevatorFF = 0.0f;


  
  }

  public static final class FunnelConstants{
    
  }

  public static final class LEDConstants{
    
  }

  public static final class ShooterArmConstants{
    
  }

  public static final class ShooterPivotConstants{
    
  }
}