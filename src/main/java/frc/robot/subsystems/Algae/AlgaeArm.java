package frc.robot.subsystems.Algae;

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

public class AlgaeArm extends SubsystemBase {

    public float algaeArmDesiredAngle;

    private SparkMax algaeArmMotor = new SparkMax(AlgaeArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController algaeArmController = algaeArmMotor.getClosedLoopController();

    private AbsoluteEncoder algaeArmEncoder = algaeArmMotor.getAbsoluteEncoder();

    public AlgaeArm() {

        algaeArmMotor.configure(Configs.AlgaeArm.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        algaeArmDesiredAngle = NormalizeAngle((float)(algaeArmEncoder.getPosition()*360));
        }




    public void setLeftInitial() {
        algaeArmDesiredAngle = (AlgaeArmConstants.);
    }
    public void setRightInitial() {
        algaeArmDesiredAngle = (AlgaeArmConstants.);
    }
    public void setCenter() {
        algaeArmDesiredAngle = (AlgaeArmConstants.);
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
        float newAngle = angle - AlgaeArmConstants.angleOffset;
 
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

        float scale = AlgaeArmConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(algaeArmDesiredAngle + amount * scale, AlgaeArmConstants.minAngle, AlgaeArmConstants.maxAngle);

        algaeArmDesiredAngle = f;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Arm Desired Angle", algaeArmDesiredAngle);
        SmartDashboard.putNumber("Algae Arm Current Angle", NormalizeAngle((float)algaeArmEncoder.getPosition()*360));
    }
}