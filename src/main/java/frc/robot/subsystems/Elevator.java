package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

    
        private RelativeEncoder elevatorEncoder;
        private SparkClosedLoopController elevatorController;
    
        public Elevator() {

            desiredPosition = 0.0f;
    
            elevatorMax = Constants.ElevatorConstants.maximumExtension;
            elevatorMin = Constants.ElevatorConstants.minimumExtension;

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
            this.setDesiredPosition(this.desiredPosition + amount * scale);
         }
    
        public void setClimb() {
            this.setDesiredPosition(this.climbPose);     
            
        }
        public void setGroundPose() {
            this.setDesiredPosition(this.groundPickupPose);       
        }
        public void setHomeState() {
            this.setDesiredPosition(this.homeStatePose);
        }
    
        public void setAmpPose() {
            this.setDesiredPosition(this.ampScorePose);       
        }
    
        //Commands
        public Command homeStateCommand()
        {
            Command command = new InstantCommand(()-> this.setHomeState(), this);
            return command;
        }
        public Command fullRetractCommand()
        {
            Command command = new InstantCommand(()-> this.setFullRetract(), this);
            return command;
        }
    
        public Command ampScoreCommand()
        {
            Command command = new InstantCommand(()-> this.setAmpPose(), this);
            return command;
        }
    
        public Command groundScoreCommand()
        {
            Command command = new InstantCommand(()-> this.setGroundPose(), this);
            return command;
        }
    
        public Command setClimbCommand()
        {
            Command command = new InstantCommand(()-> this.setClimb(), this);
            return command;
        }
        
        @Override
        public void periodic() {
            elevatorController.setReference(desiredPosition, ControlType.kMAXMotionPositionControl);
        }
    
    }
