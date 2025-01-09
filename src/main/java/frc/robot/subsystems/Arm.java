package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.simulation.SimDeviceDataJNI.SimDeviceInfo;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {

  private final DCMotor armGearbox = DCMotor.getNeo550(1);

    private SparkMax armMotor;

    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
        armGearbox,
          Constants.ArmGearboxRatio,
          Constants.ArmGearboxRatio,
          Constants.ArmGearboxRatio,
          Constants.ArmGearboxRatio,
          Constants.ArmGearboxRatio,
          true,
          0,
          0.01,
          0.0);

}
