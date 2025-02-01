package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

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
        climberMotor.set(ClimberConstants.extend);
    }

    private void retract() {
        climberMotor.set(ClimberConstants.retract);
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
}