package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ShooterArmConstants;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterArm extends SubsystemBase {

    public float shooterArmDesiredAngle;

    private SparkMax shooterArmMotor = new SparkMax(ShooterArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterArmController = shooterArmMotor.getClosedLoopController();

    private AbsoluteEncoder shooterArmEncoder = shooterArmMotor.getAbsoluteEncoder();

    public ShooterArm() {

        shooterArmMotor.configure(Configs.ShooterArm.shooterArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition() + ShooterArmConstants.angleOffset);
        }


    private void setScore() {
        shooterArmDesiredAngle = ShooterArmConstants.scoreAngle + ShooterArmConstants.angleOffset;
    }
    private void setRightInitial() {
        shooterArmDesiredAngle = ShooterArmConstants. + ShooterArmConstants.angleOffset;
    }
    private void setCenter() {
        shooterArmDesiredAngle = ShooterArmConstants. + ShooterArmConstants.angleOffset;
    }

    public Command shooterArmScoreCommand()
    {
        Command command = new InstantCommand(() -> setScore());
        return command;
    }

    public Command shooterPivotRightInitalCommand()
    {
        Command command = new InstantCommand(() -> this.setRightInitial());
        return command;
    }

    public Command shooterPivotCenterCommand()
    {
        Command command = new InstantCommand(() -> this.setCenter());
        return command;
    }


    public void moveAmount(final float amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterArmConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(shooterArmDesiredAngle + amount * scale, ShooterArmConstants.minAngle, ShooterArmConstants.maxAngle);

        shooterArmDesiredAngle = f;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Arm Desired Angle", shooterArmDesiredAngle);
        SmartDashboard.putNumber("Shooter Arm Current Angle", (float)shooterArmEncoder.getPosition() + ShooterArmConstants.angleOffset);
    }
}