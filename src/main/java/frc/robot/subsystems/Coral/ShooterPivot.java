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
import frc.robot.Configs;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {

    public float shooterPivotDesiredAngle;

    public static boolean isStraight;

    private SparkMax shooterPivotMotor = new SparkMax(ShooterPivotConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterPivotController = shooterPivotMotor.getClosedLoopController();

    private AbsoluteEncoder shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder();

    public ShooterPivot() {

        shooterPivotMotor.configure(Configs.ShooterPivot.shooterPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        isStraight = true;

        shooterPivotDesiredAngle = (float)shooterPivotEncoder.getPosition();
        }

    private void setLeftInitial() {
        shooterPivotDesiredAngle = ShooterPivotConstants.leftAngleInitial;
        isStraight = false;
    }
    private void setRightInitial() {
        shooterPivotDesiredAngle = ShooterPivotConstants.rightAngleInitial;
        isStraight = false;
    }
    private void setCenter() {
        shooterPivotDesiredAngle = ShooterPivotConstants.centerAngle;
        isStraight = true;
    }

    public Command setLeftInitialCommand()
    {
        Command command = new InstantCommand(()-> this.setLeftInitial());
        return command;
    }

    public Command setRightInitalCommand()
    {
        Command command = new InstantCommand(()-> this.setRightInitial());
        return command;
    }

    public Command setCenterCommand()
    {
        Command command = new InstantCommand(()-> this.setCenter());
        return command;
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterPivotConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(shooterPivotDesiredAngle + amount * scale, ShooterPivotConstants.min, ShooterPivotConstants.max);

        shooterPivotDesiredAngle = f;
    }

    @Override
    public void periodic() {

        shooterPivotDesiredAngle = (float)MathUtil.clamp(shooterPivotDesiredAngle, ShooterPivotConstants.min, ShooterPivotConstants.max);
        
        SmartDashboard.putNumber("Shooter Pivot Desired Angle", shooterPivotDesiredAngle);
        SmartDashboard.putNumber("Shooter Pivot Current Angle", shooterPivotEncoder.getPosition());
        
        shooterPivotController.setReference(shooterPivotDesiredAngle, ControlType.kPosition);
    }
}
