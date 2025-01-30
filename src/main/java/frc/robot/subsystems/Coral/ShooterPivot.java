package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {

    private float desiredAngle;

    private SparkMax shooterPivotMotor = new SparkMax(ShooterPivotConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterPivotController = shooterPivotMotor.getClosedLoopController();

    private AbsoluteEncoder shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder();

    public ShooterPivot() {

        shooterPivotMotor.configure(Configs.ShooterPivot.shooterPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        }

    public float NormalizeAngle(float angle) {
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

        float f = (float) MathUtil.clamp(desiredAngle + amount * scale, ShooterPivotConstants.minAngle, ShooterPivotConstants.maxAngle);

        this.desiredAngle = f;
    }

    @Override
    public void periodic() {
        System.out.println(desiredAngle);
    }
}
