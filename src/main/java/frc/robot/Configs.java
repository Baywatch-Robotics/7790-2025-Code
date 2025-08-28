package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public final class Configs {

  public static final class Elevator {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .inverted(false)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .pid(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D)
          .outputRange(-1, 1);
    }
  }
  
  public static final class ElevatorSlave {
    public static final SparkMaxConfig elevatorSlaveConfig = new SparkMaxConfig();

    static {
      elevatorSlaveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

    elevatorSlaveConfig
      .follow(ElevatorConstants.ID, true);
    }
  }


  
  public static final class EndEffector {
    public static final SparkMaxConfig endEffectorConfig = new SparkMaxConfig();
    
    static {
      endEffectorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

      endEffectorConfig
      .inverted(true);
    }
  }
  public static final class Arm {
    public static final SparkMaxConfig ArmConfig = new SparkMaxConfig();

    static {
      ArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      ArmConfig.absoluteEncoder.zeroOffset(ArmConstants.angleOffset);

      ArmConfig
          .inverted(false)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(ArmConstants.P, ArmConstants.I, ArmConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ArmConstants.maxVelocity)
          .maxAcceleration(ArmConstants.maxAcceleration)
          .allowedClosedLoopError(ArmConstants.allowedClosedLoopError);      
    }
  }

  public static final class Intake {
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

    static {
      pivotConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12);
      
      // Absolute encoder setup (match Arm style; zero offset if needed)
      pivotConfig.absoluteEncoder
        .zeroOffset(0.0);

      pivotConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
        .outputRange(-1, 1);

      rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12);
    }
  }
}