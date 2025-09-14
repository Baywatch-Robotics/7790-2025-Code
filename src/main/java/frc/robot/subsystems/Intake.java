package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final SparkMax pivotMotor  = new SparkMax(IntakeConstants.pivotMotorID, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);

    private final SparkClosedLoopController pivotPID = pivotMotor.getClosedLoopController();
    private final AbsoluteEncoder absEncoder = pivotMotor.getAbsoluteEncoder(); // Match Arm usage

    private double targetAngleRotations = IntakeConstants.stowAngleRotations;
    private boolean holdEnabled = false;
    private boolean isInitialized = false;

    // Feedforward like Arm
    private final ArmFeedforward intakeFeedforward = new ArmFeedforward(
        IntakeConstants.kS,
        IntakeConstants.kG,
        IntakeConstants.kV,
        IntakeConstants.kA
    );

    // Conversion helpers (rotations <-> radians). 1 rotation = 2π rad
    private static final double kEncoderToRadians = 2.0 * Math.PI;
    // Reference where 0 rad = horizontal; adjust if needed
    private static final double kHorizontalReferenceRad = Math.PI / 2.0;

    private double rotationsToRadians(double rotations) {
        return rotations * kEncoderToRadians;
    }

    private double toFeedforwardRadians(double encoderRotations) {
        return rotationsToRadians(encoderRotations) - kHorizontalReferenceRad;
    }

    public Intake() {
        // Configure via central Configs (avoids mixed old/new API)
        pivotMotor.configure(Configs.Intake.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(Configs.Intake.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        targetAngleRotations = IntakeConstants.stowAngleRotations;
    }

    // Pivot control
    private void setDeploy() {
        targetAngleRotations = IntakeConstants.deployAngleRotations;
    }

    private void setStow() {
        targetAngleRotations = IntakeConstants.stowAngleRotations;
    }

    private void setStart() {
        targetAngleRotations = IntakeConstants.startAngleRotations;
    }

    // Roller control
    private void setIntake() {
        rollerMotor.set(IntakeConstants.intakeSpeed);
    }

    private void setOuttake() {
        rollerMotor.set(IntakeConstants.outtakeSpeed);
    }

    private void setStop() {
        rollerMotor.set(0);
    }

    public boolean isAtTarget() {
        return Math.abs(absEncoder.getPosition() - targetAngleRotations) <= IntakeConstants.angleTolerance;
    }

    // Commands
    public Command deployCommand() { return new InstantCommand(this::setDeploy, this); }
    public Command stowCommand()   { return new InstantCommand(this::setStow, this); }
    public Command startCommand()   { return new InstantCommand(this::setStart, this); }
    public Command intakeCommand() { return new InstantCommand(this::setIntake, this); }
    public Command outtakeCommand(){ return new InstantCommand(this::setOuttake, this); }
    public Command stopCommand()   { return new InstantCommand(this::setStop, this); }

    @Override
    public void periodic() {

        if (!isInitialized) {
            targetAngleRotations = (float)(absEncoder.getPosition());
            isInitialized = true;
            holdEnabled = true;
        }

        // Compute feedforward like Arm
        double currentPositionRad = toFeedforwardRadians(absEncoder.getPosition() - IntakeConstants.feedforwardOffset);
        double currentVelocityRad = absEncoder.getVelocity() * kEncoderToRadians; // rotations/s -> rad/s
        // Use non-deprecated overload (position, velocity)
        double ffVolts = intakeFeedforward.calculate(currentPositionRad, currentVelocityRad);

        if (holdEnabled) {
            pivotPID.setReference(
                targetAngleRotations,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage
            );
        }

        SmartDashboard.putNumber("Intake FF Volts", ffVolts);
        SmartDashboard.putNumber("Intake Position (rad)", currentPositionRad);
        SmartDashboard.putNumber("Intake Velocity (rad/s)", currentVelocityRad);
        SmartDashboard.putNumber("Intake Pivot Angle", absEncoder.getPosition());
        SmartDashboard.putNumber("Intake Pivot Target", targetAngleRotations);
        SmartDashboard.putBoolean("Intake Pivot At Target", isAtTarget());
        SmartDashboard.putBoolean("Intake Holding", holdEnabled);
    }
}
