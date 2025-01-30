package frc.robot.subsystems.Algea;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgeaShooterConstants;

public class AlgeaShooter extends SubsystemBase {

    private double desiredSpeed;

    private float intake;
    private float outake;

    private SparkMax algeaShooterMotor = new SparkMax(AlgeaShooterConstants.ID, MotorType.kBrushless);

    public AlgeaShooter() {

        desiredSpeed = 0.0f;

        intake = AlgeaShooterConstants.intake;
        outake = AlgeaShooterConstants.outake;


        algeaShooterMotor.configure(
                Configs.AlgeaShooter.algeaShooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }
                
    public void setZeroSpeed() {

        this.desiredSpeed = 0.0f;
    }

    public void setIntake() {
        this.desiredSpeed = intake;
    }

    public void setOutake() {
        this.desiredSpeed = outake;
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

    @Override
    public void periodic() {
        algeaShooterMotor.set(desiredSpeed);
    }
}