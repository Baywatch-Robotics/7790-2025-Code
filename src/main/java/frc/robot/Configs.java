package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterArmConstants;
import frc.robot.Constants.ShooterPivotConstants;

public final class Configs {
  public static final class Elevator {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
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
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.maxVelocity)
          .maxAcceleration(ElevatorConstants.maxAcceleration)
          .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError);      
    }
  }


  public static final class Climber {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

      climberConfig
      .inverted(false);
    }
  }


  
  public static final class Shooter {
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the elevator motor
      shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

      shooterConfig
      .inverted(false);
    }
  }
  public static final class ShooterArm {
    public static final SparkMaxConfig shooterArmConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      shooterArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      shooterArmConfig
          .inverted(false)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(ShooterArmConstants.P, ShooterArmConstants.I, ShooterArmConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ShooterArmConstants.maxVelocity)
          .maxAcceleration(ShooterArmConstants.maxAcceleration)
          .allowedClosedLoopError(ShooterArmConstants.allowedClosedLoopError);      
    }
  }
  public static final class ShooterPivot {
    public static final SparkMaxConfig shooterPivotConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      shooterPivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      shooterPivotConfig
      .inverted(false)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(ShooterPivotConstants.P, ShooterPivotConstants.I, ShooterPivotConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ShooterPivotConstants.maxVelocity)
          .maxAcceleration(ShooterPivotConstants.maxAcceleration)
          .allowedClosedLoopError(ShooterPivotConstants.allowedClosedLoopError);      
    }
  }




  public static final class AlgaeArm {
    public static final SparkMaxConfig algaeArmConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      algaeArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      algaeArmConfig
          .inverted(false)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(AlgaeArmConstants.P, AlgaeArmConstants.I, AlgaeArmConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(AlgaeArmConstants.maxVelocity)
          .maxAcceleration(AlgaeArmConstants.maxAcceleration)
          .allowedClosedLoopError(AlgaeArmConstants.allowedClosedLoopError);      
    }
  }
  public static final class AlgaeShooter {
    public static final SparkMaxConfig algaeShooterConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the elevator motor
      algaeShooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      
      algaeShooterConfig
      .inverted(false);
    }
  }
}