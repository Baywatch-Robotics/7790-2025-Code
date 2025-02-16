package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Coral.ShooterPivot;

public class Elevator extends SubsystemBase {

    public float elevatorDesiredPosition = 0;

    private float desiredTotalHeight;

    private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    public static boolean isRaised = false;

    private RelativeEncoder m_encoder;
    
    public Elevator() {

        m_encoder = elevatorMotor.getEncoder();
        m_encoder.setPosition(0);
        elevatorMotor.configure(
                Configs.Elevator.elevatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

      
        
        System.out.println("Elevator Initialized " + m_encoder.getPosition());

    }

    private void setFullRetract() {
        elevatorDesiredPosition = 0;
        isRaised = false;
    }

    private void setL4() {
        elevatorDesiredPosition = ElevatorConstants.L4Pose;
        isRaised = true;
    }

    private void setL3() {
        elevatorDesiredPosition = ElevatorConstants.L3Pose;
        isRaised = true;
    }

    private void setL2() {
        elevatorDesiredPosition = ElevatorConstants.L2Pose;
        isRaised = true;
    }

    private void setL1() {
        elevatorDesiredPosition = ElevatorConstants.L1Pose;
        isRaised = true;
    }
    public void setPickup() {
        elevatorDesiredPosition = ElevatorConstants.pickupPose;
        isRaised = false;
    }

    // Commands
    public Command fullElevatorRetractCommand() {
        Command command = new InstantCommand(() -> setFullRetract());
        return command;
    }

    public Command setElevatorL4Command() {
        Command command = new InstantCommand(() -> setL4());
        return command;
    }

    public Command setElevatorL3Command() {
        Command command = new InstantCommand(() -> setL3());
        return command;
    }

    public Command setElevatorL2Command() {
        Command command = new InstantCommand(() -> setL2());
        return command;
    }

    public Command setElevatorL1Command() {
        Command command = new InstantCommand(() -> setL1());
        return command;
    }
    public Command setElevatorPickupCommand() {
        Command command = new InstantCommand(() -> setPickup());
        return command;
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ElevatorConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(elevatorDesiredPosition + amount * scale, ElevatorConstants.min, ElevatorConstants.max);

        elevatorDesiredPosition = f;

        System.out.println("Elevator Desired Position: " + elevatorDesiredPosition);
    }

    @Override
    public void periodic() {

         //Code for inverse kinematics
        //if(!ShooterPivot.isStraight) {
        //float ang;

        //float l;

        //ang = ShooterPivot.shooterPivotDesiredAngle * (360/25);

        //l = Constants.armLength;

        //desiredTotalHeight = elevatorDesiredPosition + l * (float)Math.sin(Units.degreesToRadians(ang));


        //}
        //else{
        //desiredTotalHeight = elevatorDesiredPosition;  
        //}
        desiredTotalHeight = elevatorDesiredPosition; 

        //System.out.println(desiredTotalHeight);
        
        desiredTotalHeight = (float)MathUtil.clamp(desiredTotalHeight, ElevatorConstants.min, ElevatorConstants.max);
        
        SmartDashboard.putNumber("Elevator Desired Height", desiredTotalHeight);
        SmartDashboard.putNumber("Elevator Current Height", elevatorMotor.getEncoder().getPosition());

        elevatorClosedLoopController.setReference(desiredTotalHeight, ControlType.kMAXMotionPositionControl);
    }
}