package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final SparkMax leftMotor  = new SparkMax(IndexerConstants.leftMotorID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(IndexerConstants.rightMotorID, MotorType.kBrushless);

  private double commandedSpeed = 0.0;

  // NEW: indexing detection state
  private boolean coralIndexed = false;
  private double debounceStart = -1.0;

  // NEW: optional beam-break sensor
  private final DigitalInput beamBreak =
      (IndexerConstants.useBeamBreak) ? new DigitalInput(IndexerConstants.beamBreakDIOPort) : null;

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

  // NEW: raw sensor query (true = coral present)
  private boolean isBeamBroken() {
    if (beamBreak == null) return false;
    boolean raw = beamBreak.get();               // WPILib returns true when circuit is HIGH
    boolean present = IndexerConstants.beamBreakNormallyClosed ? !raw : !raw;
    // Adjust logic if wiring differs; present = beam blocked
    return present;
  }

  // OPTIONAL: public accessor
  public boolean sensorPresent() {
    return beamBreak != null;
  }

  // NEW: expose trigger used by CommandFactory
  public BooleanSupplier coralIndexedTrigger() {
    return () -> coralIndexed;
  }

  // NEW: allow manual reset between cycles
  public void resetCoralIndexed() {
    coralIndexed = false;
    debounceStart = -1.0;
  }

  // NEW: command wrapper for reset
  public Command resetIndexedCommand() {
    return new InstantCommand(this::resetCoralIndexed, this);
  }

  public Command indexCommand()   { return new InstantCommand(this::runForward, this); }
  public Command reverseCommand() { return new InstantCommand(this::runReverse, this); }
  public Command stopCommand()    { return new InstantCommand(this::stop, this); }

  @Override
  public void periodic() {
    // (Optional) telemetry can be added later

    // Determine detection source
    boolean detectedNow;
    if (beamBreak != null) {
      // Sensor-based
      detectedNow = isBeamBroken();
    } else {
      // Current-based fallback
      double leftI = leftMotor.getOutputCurrent();
      double rightI = rightMotor.getOutputCurrent();
      double avgI = (leftI + rightI) / 2.0;

      if (!coralIndexed) {
        if (avgI >= IndexerConstants.indexCurrentThreshold) {
          if (debounceStart < 0) {
            debounceStart = Timer.getFPGATimestamp();
          } else if (Timer.getFPGATimestamp() - debounceStart >= IndexerConstants.indexDebounceTime) {
            coralIndexed = true;
          }
        } else {
          debounceStart = -1.0;
        }
      } else {
        if (avgI <= IndexerConstants.indexReleaseHysteresis) {
          // leave latched unless manual reset
        }
      }
      detectedNow = coralIndexed;
    }

    // Unified debounce/latch layer (applies to sensor path only)
    if (beamBreak != null) {
      if (!coralIndexed) {
        if (detectedNow) {
          if (debounceStart < 0) {
            debounceStart = Timer.getFPGATimestamp();
          } else if (Timer.getFPGATimestamp() - debounceStart >= IndexerConstants.indexDebounceTime) {
            coralIndexed = true;
          }
        } else {
          debounceStart = -1.0;
        }
      } else {
        // Optionally add release hysteresis for sensor if desired
      }
    }

    // (Optional) SmartDashboard logging
    // SmartDashboard.putBoolean("Indexer Coral Indexed", coralIndexed);
    // SmartDashboard.putNumber("Indexer Avg Current", avgI);
  }
}
