package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.ArmConstants;

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
          .outputRange(-1, 1);
    }
  }
  
  public static final class ElevatorSlave {
    public static final SparkMaxConfig elevatorSlaveConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      elevatorSlaveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

    elevatorSlaveConfig
      .follow(ElevatorConstants.ID, true);
    }
  }

  

  public static final class Climber {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the climber motor
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80).voltageCompensation(12);

      climberConfig
      .inverted(false)
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control
      .pid(ClimberConstants.P, ClimberConstants.I, ClimberConstants.D)
      .outputRange(-1, 1)
      .maxMotion
      // Set MAXMotion parameters for position control
      .maxVelocity(ClimberConstants.maxVelocity)
      .maxAcceleration(ClimberConstants.maxAcceleration);
    }
  }


  
  public static final class EndEffector {
    public static final SparkMaxConfig endEffectorConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the elevator motor
      endEffectorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

      endEffectorConfig
      .inverted(true);
    }
  }
  public static final class Arm {
    public static final SparkMaxConfig ArmConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
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

  public static final class AlgaeArm {
    public static final SparkMaxConfig algaeArmConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      algaeArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      algaeArmConfig.absoluteEncoder.zeroOffset(AlgaeArmConstants.angleOffset);

      algaeArmConfig
          
          .inverted(true)
          
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
  public static final class Algae {
    public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the elevator motor
      algaeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      
      algaeConfig
      .inverted(false);
    }
  }

  public static final class Funnel {
    public static final SparkMaxConfig funnelConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the funnel motor - 20 amp limit as requested
      funnelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
      
      // Configure absolute encoder
      funnelConfig.absoluteEncoder.zeroOffset(FunnelConstants.angleOffset);

      funnelConfig
          .inverted(true)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .pid(FunnelConstants.P, FunnelConstants.I, FunnelConstants.D)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(FunnelConstants.maxVelocity)
          .maxAcceleration(FunnelConstants.maxAcceleration)
          .allowedClosedLoopError(FunnelConstants.allowedClosedLoopError);      
    }
  }
}