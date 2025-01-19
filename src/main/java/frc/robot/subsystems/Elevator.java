package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {


        
        private double desiredPosition;

        private float elevatorMax;
        private float elevatorMin;

        //List different poses here
        private float L4Pose;
        private float L3Pose;
        private float L2Pose;
        private float L1Pose;

          private SparkFlex elevatorMotor =
      new SparkFlex(ElevatorConstants.elevatorID, MotorType.kBrushless);

  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    

        public Elevator() {

            desiredPosition = 0.0f;
    
            elevatorMax = Constants.ElevatorConstants.elevatorMax;
            elevatorMin = Constants.ElevatorConstants.elevatorMin;

            //different positions for scoring below here
            
            

            SparkMaxConfig config = new SparkMaxConfig();

            config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            config.encoder
            .positionConversionFactor(1);
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .maxOutput(Constants.ElevatorConstants.elevatorMaxSpeed)
            .minOutput(-Constants.ElevatorConstants.elevatorMaxSpeed)
            .pidf(Constants.ElevatorConstants.elevatorP, Constants.ElevatorConstants.elevatorI, Constants.ElevatorConstants.elevatorD, Constants.ElevatorConstants.elevatorFF);
    
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }
        
        public void setDesiredPosition(final float desiredPose) {
    
            float newDesiredPose = (float)MathUtil.clamp(desiredPose, elevatorMin, elevatorMax);
    
    
    
            this.desiredPosition = (newDesiredPose);
        }
    
        public void setFullRetract(){
    
            this.desiredPosition = 0;
        }
    
        public void setL4() {
            setDesiredPosition(L4Pose);
            moveToSetpoint(desiredPosition);     
        }
        public void setL3() {
            setDesiredPosition(L3Pose);
            moveToSetpoint(desiredPosition);      
        }
        public void setL2() {
            setDesiredPosition(L2Pose);
            moveToSetpoint(desiredPosition);
        }
        public void setL1() {
            setDesiredPosition(L1Pose);   
            moveToSetpoint(desiredPosition);    
        }
    
        //Commands
        public Command fullRetractCommand()
        {
            Command command = new InstantCommand(()-> setFullRetract());
            return command;
        }
    
        public Command setL4Command()
        {
            Command command = new InstantCommand(()-> setL4());
            return command;
        }
    
        public Command setL3Command()
        {
            Command command = new InstantCommand(()-> setL3());
            return command;
        }
    
        public Command setL2Command()
        {
            Command command = new InstantCommand(()-> setL2());
            return command;
        }
        public Command setL1Command()
        {
            Command command = new InstantCommand(()-> setL1());
            return command;
        }
        
        private void moveToSetpoint(double desiredPositon) {
        elevatorController.setReference(desiredPosition, ControlType.kMAXMotionPositionControl);
          }
    }