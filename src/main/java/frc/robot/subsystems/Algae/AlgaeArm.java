package frc.robot.subsystems.Algae;

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
import frc.robot.Constants.AlgaeArmConstants;

public class AlgaeArm extends SubsystemBase {

    public double algaeArmDesiredAngle;

    private SparkMax algaeArmMotor = new SparkMax(AlgaeArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController algaeArmController = algaeArmMotor.getClosedLoopController();

    private AbsoluteEncoder algaeArmEncoder = algaeArmMotor.getAbsoluteEncoder();

    public AlgaeArm() {

        algaeArmMotor.configure(Configs.AlgaeArm.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeArmDesiredAngle = (float)(algaeArmEncoder.getPosition() + AlgaeArmConstants.angleOffset);
        }

    // Define desired positions for the arm
    private void stowUp() {
        algaeArmDesiredAngle = AlgaeArmConstants.stowedUpAngle + AlgaeArmConstants.angleOffset;
    }
    private void straightOut() {
        algaeArmDesiredAngle = AlgaeArmConstants.straightOutAngle + AlgaeArmConstants.angleOffset;
    }
    private void groundIntake() {
        algaeArmDesiredAngle = AlgaeArmConstants.groundIntakeAngle + AlgaeArmConstants.angleOffset;
    }

    // Commands to move the arm to the desired positions
    public Command algaeArmStowUpCommand()
    {
        Command command = new InstantCommand(() -> stowUp());
        return command;
    }

    public Command algaeArmStraightOutCommand()
    {
        Command command = new InstantCommand(() -> straightOut());
        return command;
    }

    public Command algaeArmGroundIntakeCommand()
    {
        Command command = new InstantCommand(() -> groundIntake());
        return command;
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        double scale = AlgaeArmConstants.manualMultiplier;

        double f = MathUtil.clamp(algaeArmDesiredAngle + amount * scale, AlgaeArmConstants.min, AlgaeArmConstants.max);

        algaeArmDesiredAngle = f;
    }

    @Override
    public void periodic() {

        algaeArmDesiredAngle =MathUtil.clamp(algaeArmDesiredAngle, AlgaeArmConstants.min, AlgaeArmConstants.max);

        
        SmartDashboard.putNumber("Algae Arm Desired Angle", algaeArmDesiredAngle);
        SmartDashboard.putNumber("Algae Arm Current Angle", algaeArmEncoder.getPosition() + AlgaeArmConstants.angleOffset);
        SmartDashboard.putNumber("Algae Arm 2 Current Angle", algaeArmMotor.getEncoder().getPosition() + AlgaeArmConstants.angleOffset);
      

       
        
        algaeArmController.setReference(algaeArmDesiredAngle, ControlType.kMAXMotionPositionControl);

    }
}