package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeShooterConstants;

public class AlgaeShooter extends SubsystemBase {

    private SparkMax algaeShooterMotor = new SparkMax(AlgaeShooterConstants.ID, MotorType.kBrushless);

    public static boolean algaeLoaded;
    private boolean isLoading;
    
    // Debounce timer variables
    private double currentAboveThresholdStartTime = 0;
    private double currentBelowThresholdStartTime = 0;
    private final double DEBOUNCE_TIME = 0.5; // 0.5 seconds for debouncing

    public AlgaeShooter() {
        algaeShooterMotor.configure(
                Configs.AlgaeShooter.algaeShooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        algaeLoaded = false;
        isLoading = false;
    }
                
    // Method to get the current draw from the motor
    private double getCurrentDraw() {
        return algaeShooterMotor.getOutputCurrent();
    }

    // Method to check if algae is loaded based on current draw with debouncing
    public boolean checkAlgaeLoaded() {
        double currentDraw = getCurrentDraw();
        double currentTime = Timer.getFPGATimestamp();
        
        if (currentDraw > AlgaeShooterConstants.currentThreshold) {
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

    private void setZeroSpeed() {
        isLoading = false;
        algaeShooterMotor.set(0);
    }

    private void setIntake() {
        isLoading = true;
        algaeShooterMotor.set(AlgaeShooterConstants.intake);
    }

    private void setOutake() {
        algaeShooterMotor.set(AlgaeShooterConstants.outake);
    }

    // Commands
    public Command algaeShooterZeroSpeedCommand() {
        return new InstantCommand(() -> setZeroSpeed());
    }

    public Command algaeShooterIntakeCommand() {
        return new InstantCommand(() -> setIntake());
    }

    public Command algaeShooterOutakeCommand() {
        return new InstantCommand(() -> setOutake());
    }
    
    @Override
    public void periodic() {
        checkAlgaeLoaded();
        algaeLoadedTrigger();
        
        if(algaeLoaded && isLoading) {
            algaeShooterZeroSpeedCommand().schedule(); // Stop the motor when algae is loaded
        }

        SmartDashboard.putNumber("Algae Current Draw", getCurrentDraw());
        SmartDashboard.putBoolean("Algae Loaded", algaeLoaded);
    }
}