package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeArmConstants;

public class AlgaeArm extends SubsystemBase {

    public double algaeArmDesiredAngle;

    private boolean isInitialized = false;
    
    // Algae piece detection
    public static boolean algaeLoaded = false;
    private boolean isIntaking = false;
    
    // Debounce timer variables
    private double currentAboveThresholdStartTime = 0;
    private double currentBelowThresholdStartTime = 0;
    private final double DEBOUNCE_TIME = 0.5; // 0.5 seconds for debouncing

    private SparkMax algaeArmMotor = new SparkMax(AlgaeArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController algaeArmController = algaeArmMotor.getClosedLoopController();

    private AbsoluteEncoder algaeArmEncoder = algaeArmMotor.getAbsoluteEncoder();

    public AlgaeArm() {
        algaeArmMotor.configure(Configs.AlgaeArm.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to get the current draw from the motor
    private double getCurrentDraw() {
        return algaeArmMotor.getOutputCurrent();
    }

    // Method to check if algae is loaded based on current draw with debouncing
    public boolean checkAlgaeLoaded() {
        double currentDraw = getCurrentDraw();
        double currentTime = Timer.getFPGATimestamp();
        
        if (currentDraw > AlgaeArmConstants.currentThreshold) {
            // Reset the "below threshold" timer since we're above threshold
            currentBelowThresholdStartTime = currentTime;
            
            // If this is the first time we're above threshold, start timing
            if (currentAboveThresholdStartTime == 0) {
                currentAboveThresholdStartTime = currentTime;
            }
            
            // Check if we've been above threshold long enough
            if (currentTime - currentAboveThresholdStartTime >= DEBOUNCE_TIME) {
                return algaeLoaded = true;
            }
        } else {
            // Reset the "above threshold" timer since we're below threshold
            currentAboveThresholdStartTime = 0;
            
            // If this is the first time we're below threshold, start timing
            if (currentBelowThresholdStartTime == 0) {
                currentBelowThresholdStartTime = currentTime;
            }
            
            // Check if we've been below threshold long enough
            if (currentTime - currentBelowThresholdStartTime >= DEBOUNCE_TIME) {
                return algaeLoaded = false;
            }
        }
        
        // Return the current state if we haven't debounced yet
        return algaeLoaded;
    }
    
    public Trigger algaeLoadedTrigger() { 
        return new Trigger(this::checkAlgaeLoaded);
    }

    // Define desired positions for the arm
    private void stowUp() {
        algaeArmDesiredAngle = AlgaeArmConstants.stowedUpAngle;
        isIntaking = false;
    }
    
    private void straightOut() {
        algaeArmDesiredAngle = AlgaeArmConstants.straightOutAngle;
        isIntaking = false;
    }
    
    private void groundIntake() {
        algaeArmDesiredAngle = AlgaeArmConstants.groundIntakeAngle;
        isIntaking = true;
    }
    
    private void holdPosition() {
        algaeArmDesiredAngle = AlgaeArmConstants.holdAngle;
        isIntaking = false;
    }

    // Commands to move the arm to the desired positions
    public Command algaeArmStowUpCommand() {
        return new InstantCommand(() -> stowUp());
    }

    public Command algaeArmStraightOutCommand() {
        return new InstantCommand(() -> straightOut());
    }

    public Command algaeArmGroundIntakeCommand() {
        return new InstantCommand(() -> groundIntake());
    }
    
    public Command algaeArmHoldCommand() {
        return new InstantCommand(() -> holdPosition());
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

        if (!isInitialized) {
            algaeArmDesiredAngle = AlgaeArmConstants.stowedUpAngle;
            isInitialized = true;
        }
        
        // Check if algae is loaded
        checkAlgaeLoaded();
        
        // If we have algae and we're in intake position, move to hold position
        if (algaeLoaded && isIntaking) {
            holdPosition();
        }
        
        algaeArmDesiredAngle = MathUtil.clamp(algaeArmDesiredAngle, AlgaeArmConstants.min, AlgaeArmConstants.max);

        SmartDashboard.putNumber("Algae Arm Desired Angle", algaeArmDesiredAngle);
        SmartDashboard.putNumber("Algae Arm Current Angle", algaeArmEncoder.getPosition());
        SmartDashboard.putNumber("Algae Arm Current Draw", getCurrentDraw());
        SmartDashboard.putBoolean("Algae Loaded", algaeLoaded);

        algaeArmController.setReference(algaeArmDesiredAngle, ControlType.kPosition);

    }
}