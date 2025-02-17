package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterArmConstants;

public class Climber extends SubsystemBase {

    private SparkMax climberMotor = new SparkMax(ClimberConstants.ID, MotorType.kBrushless);

    public Climber() {
        climberMotor.configure(
                Configs.Climber.climberConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void setZero() {
        climberMotor.set(0);
    }

    private void extend() {
        climberMotor.set(ClimberConstants.extendSpeed);
    }

    private void retract() {
        climberMotor.set(ClimberConstants.retractSpeed);
    }

    // Commands
    public Command climberExtendCommand() {
        Command command = new InstantCommand(() -> extend());
        return command;
    }

    public Command climberRetractCommand() {
        Command command = new InstantCommand(() -> retract());
        return command;
    }

    public Command climberStopCommand() {
        Command command = new InstantCommand(() -> setZero());
        return command;
    }

        public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            climberMotor.set(0);
        }

        climberMotor.set(MathUtil.clamp(amount, -1, 1));
    }
}