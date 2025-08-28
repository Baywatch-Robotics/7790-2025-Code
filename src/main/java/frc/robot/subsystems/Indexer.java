package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final SparkMax leftMotor  = new SparkMax(IndexerConstants.leftMotorID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(IndexerConstants.rightMotorID, MotorType.kBrushless);

  private double commandedSpeed = 0.0;

  public Indexer() {
    leftMotor.configure (Configs.Indexer.leftConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(Configs.Indexer.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void runForward() {
    commandedSpeed = IndexerConstants.runSpeed;
    leftMotor.set(commandedSpeed);
    rightMotor.set(commandedSpeed); // Right inverted in config -> opposing directions
  }

  private void runReverse() {
    commandedSpeed = IndexerConstants.reverseSpeed;
    leftMotor.set(commandedSpeed);
    rightMotor.set(commandedSpeed);
  }

  private void stop() {
    commandedSpeed = 0.0;
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public Command indexCommand()   { return new InstantCommand(this::runForward, this); }
  public Command reverseCommand() { return new InstantCommand(this::runReverse, this); }
  public Command stopCommand()    { return new InstantCommand(this::stop, this); }

  @Override
  public void periodic() {
    // (Optional) telemetry can be added later
  }
}
