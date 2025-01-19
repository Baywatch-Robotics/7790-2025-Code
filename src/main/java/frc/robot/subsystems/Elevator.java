package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {


        private SparkMax elevatorMotor;

        private float desiredPosition;

        private float elevatorMax;
        private float elevatorMin;

        //List different poses here
        private float L4Pose;
        private float L3Pose;
        private float L2Pose;
        private float L1Pose;

        private SparkClosedLoopController elevatorController;
    
        public Elevator() {

            desiredPosition = 0.0f;
    
            elevatorMax = Constants.ElevatorConstants.elevatorMax;
            elevatorMin = Constants.ElevatorConstants.elevatorMin;

            //different positions for scoring below here
            
            elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorID, SparkLowLevel.MotorType.kBrushless);
            

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
    
        public void extendAmount(final float amount) {
    
            
            if (Math.abs(amount)<0.2){
                return;
            }
    
            float scale = 0.2f;
            setDesiredPosition(desiredPosition + amount * scale);
         }
    
        public void setL4() {
            setDesiredPosition(L4Pose);     
        }
        public void setL3() {
            setDesiredPosition(L3Pose);       
        }
        public void setL2() {
            setDesiredPosition(L2Pose);
        }
        public void setL1() {
            setDesiredPosition(L1Pose);       
        }
    
        //Commands
        public Command fullRetractCommand()
        {
            Command command = new InstantCommand(()-> this.setFullRetract(), this);
            return command;
        }
    
        public Command setL4Command()
        {
            Command command = new InstantCommand(()-> this.setL4(), this);
            return command;
        }
    
        public Command setL3Command()
        {
            Command command = new InstantCommand(()-> this.setL3(), this);
            return command;
        }
    
        public Command setL2Command()
        {
            Command command = new InstantCommand(()-> this.setL2(), this);
            return command;
        }
        public Command setL1Command()
        {
            Command command = new InstantCommand(()-> this.setL1(), this);
            return command;
        }
        
        @Override
        public void periodic() {
            elevatorController.setReference(desiredPosition, ControlType.kMAXMotionPositionControl);
        }
    }