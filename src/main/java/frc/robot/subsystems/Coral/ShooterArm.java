package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterArmConstants;

public class ShooterArm extends SubsystemBase {

    public float shooterArmDesiredAngle;
    
    public boolean isInitialized = false;

    private SparkMax shooterArmMotor = new SparkMax(ShooterArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterArmController = shooterArmMotor.getClosedLoopController();

    private AbsoluteEncoder shooterArmEncoder = shooterArmMotor.getAbsoluteEncoder();

    public ShooterArm() {

        shooterArmMotor.configure(Configs.ShooterArm.shooterArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
        }


    private void setScoreLOW() {
        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleLOW;
    }
    private void setScoreHIGH() {
        shooterArmDesiredAngle = ShooterArmConstants.scoreAngleHIGH;
    }
    private void setLoad() {
        shooterArmDesiredAngle = ShooterArmConstants.loadAngle;
    }
    private void setScoreL1() {
        shooterArmDesiredAngle = ShooterArmConstants.L1Angle;
    }

    public Command shooterArmScoreLOWCommand()
    {
        Command command = new InstantCommand(() -> setScoreLOW());
        return command;
    }
    public Command shooterArmScoreHIGHCommand()
    {
        Command command = new InstantCommand(() -> this.setScoreLOW());
        return command;
    }

    public Command shooterArmLoadCommand()
    {
        Command command = new InstantCommand(() -> this.setLoad());
        return command;
    }

    public Command shooterArmScoreL1Command()
    {
        Command command = new InstantCommand(() -> this.setScoreL1());
        return command;
    }

    public Trigger isClearToElevate() {
        return new Trigger(() -> shooterArmEncoder.getPosition() >= 0.5);
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterArmConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(shooterArmDesiredAngle + amount * scale, ShooterArmConstants.min, ShooterArmConstants.max);

        shooterArmDesiredAngle = f;
    }

    @Override
    public void periodic() {
        
        if (!isInitialized) {
            shooterArmDesiredAngle = (float)(shooterArmEncoder.getPosition());
            isInitialized = true;
        }
        
        isClearToElevate();
        
        shooterArmDesiredAngle = (float)MathUtil.clamp(shooterArmDesiredAngle, ShooterArmConstants.min, ShooterArmConstants.max);

        SmartDashboard.putNumber("Shooter Arm Desired Angle", shooterArmDesiredAngle);
        SmartDashboard.putNumber("Shooter Arm Current Angle", (float)shooterArmEncoder.getPosition());
        
        shooterArmController.setReference(shooterArmDesiredAngle, ControlType.kPosition);

    }
}