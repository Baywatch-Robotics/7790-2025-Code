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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {

    public float shooterPivotDesiredAngle;

    private SparkMax shooterPivotMotor = new SparkMax(ShooterPivotConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterPivotController = shooterPivotMotor.getClosedLoopController();

    private AbsoluteEncoder shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder();

    public ShooterPivot() {

        shooterPivotMotor.configure(Configs.ShooterPivot.shooterPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterPivotDesiredAngle = NormalizeAngle((float)(shooterPivotEncoder.getPosition()));
        }




    private void setLeftInitial() {
        shooterPivotDesiredAngle = (ShooterPivotConstants.leftAngleInitial);
    }
    private void setRightInitial() {
        shooterPivotDesiredAngle = (ShooterPivotConstants.rightAngleInitial);
    }
    private void setCenter() {
        shooterPivotDesiredAngle = (ShooterPivotConstants.centerAngle);
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





    private float NormalizeAngle(float angle) {
        float newAngle = angle - ShooterPivotConstants.angleOffset;

        while (newAngle > 180) {
            newAngle -= 360;
        }

        while (newAngle < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public void moveAmount(final float amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterPivotConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(shooterPivotDesiredAngle + amount * scale, ShooterPivotConstants.minAngle, ShooterPivotConstants.maxAngle);

        shooterPivotDesiredAngle = f;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Desired Angle", shooterPivotDesiredAngle);
        SmartDashboard.putNumber("Shooter Pivot Current Angle", NormalizeAngle((float)shooterPivotEncoder.getPosition()));
    
        shooterPivotDesiredAngle = (float)MathUtil.clamp(shooterPivotDesiredAngle, ShooterPivotConstants.min, ShooterPivotConstants.max);
        
        shooterPivotController.setReference(shooterPivotDesiredAngle/360, ControlType.kMAXMotionPositionControl);
    }
}
