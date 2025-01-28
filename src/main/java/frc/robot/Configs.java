package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;

public final class Configs {
  public static final class Elevator {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .pid(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.maxVelocity)
          .maxAcceleration(ElevatorConstants.maxAcceleration)
          .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError);      
    }
  }
  public static final class ShooterArm {
    public static final SparkMaxConfig shooterArmConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      shooterArmConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      shooterArmConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.maxVelocity)
          .maxAcceleration(ElevatorConstants.maxAcceleration)
          .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError);      
    }
  }
}