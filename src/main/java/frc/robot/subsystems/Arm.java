package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.simulation.SimDeviceDataJNI.SimDeviceInfo;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {


    private SparkMax armMotor;

    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private final double estimateMOI();

    private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
        ArmConstants.armGearboxMotor,
      ArmConstants.armGearboxRatio,
      ArmConstants.armGearboxRatio,
      ArmConstants.armGearboxRatio,
      ArmConstants.armGearboxRatio,
      ArmConstants.armGearboxRatio,
      true,
      0,
      0,
      0.0);

}
