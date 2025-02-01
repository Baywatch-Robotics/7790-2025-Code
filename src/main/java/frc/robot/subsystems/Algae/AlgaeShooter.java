package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeShooterConstants;

public class AlgaeShooter extends SubsystemBase {

    private SparkMax algaeShooterMotor = new SparkMax(AlgaeShooterConstants.ID, MotorType.kBrushless);

    public AlgaeShooter() {
        algaeShooterMotor.configure(
                Configs.AlgaeShooter.algaeShooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }
                
    public void setZeroSpeed() {
        algaeShooterMotor.set(0);
    }

    public void setIntake() {
        algaeShooterMotor.set(AlgaeShooterConstants.intake);
    }

    public void setOutake() {
        algaeShooterMotor.set(AlgaeShooterConstants.outake);
    }

    // Commands
    public Command setZeroSpeedCommand() {
        Command command = new InstantCommand(() -> setZeroSpeed());
        return command;
    }

    public Command setIntakeCommand() {
        Command command = new InstantCommand(() -> setIntake());
        return command;
    }

    public Command setOutakeCommand() {
        Command command = new InstantCommand(() -> setOutake());
        return command;
    }
}