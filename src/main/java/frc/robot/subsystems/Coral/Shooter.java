package frc.robot.subsystems.Coral;

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
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private SparkMax shooterMotor = new SparkMax(ShooterConstants.ID, MotorType.kBrushless);

    public static boolean coralLoaded;

    private boolean isLoading;
    
    // Debounce timer variables
    private double currentAboveThresholdStartTime = 0;
    private double currentBelowThresholdStartTime = 0;
    private final double DEBOUNCE_TIME = 0.25;

    public Shooter() {
        shooterMotor.configure(
                Configs.Shooter.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        coralLoaded = false;
        isLoading = false;
    }

    // Method to get the current draw from the motor
    private double getCurrentDraw() {
        return shooterMotor.getOutputCurrent();
    }

    // Method to check if the coral is loaded based on current draw with debouncing
    public boolean checkCoralLoaded() {
        double currentDraw = getCurrentDraw();
        double currentTime = Timer.getFPGATimestamp();
        
        if (currentDraw > ShooterConstants.currentThreshold) {
            // Reset the "below threshold" timer since we're above threshold
            currentBelowThresholdStartTime = currentTime;
            
            // If this is the first time we're above threshold, start timing
            if (currentAboveThresholdStartTime == 0) {
                currentAboveThresholdStartTime = currentTime;
            }
            
            // Check if we've been above threshold long enough
            if (currentTime - currentAboveThresholdStartTime >= DEBOUNCE_TIME) {
                return coralLoaded = true;
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
                return coralLoaded = false;
            }
        }
        
        // Return the current state if we haven't debounced yet
        return coralLoaded;
    }
    
    public Trigger coralLoadedTrigger(){ 
        return new Trigger(this::checkCoralLoaded);
    }
    
    private void setZeroSpeed() {
        isLoading = false;
        shooterMotor.set(0);
    }

    private void setIntake() {
        isLoading = true;
        shooterMotor.set(ShooterConstants.intake);
    }

    private void setOutake() {
        shooterMotor.set(ShooterConstants.outake);
    }

    // Commands
    public Command shooterZeroSpeedCommand() {
        Command command = new InstantCommand(() -> setZeroSpeed());
        
        return command;
    }

    public Command shooterIntakeCommand() {
        Command command = new InstantCommand(() -> setIntake());
        return command;
    }

    public Command shooterOutakeCommand() {
        Command command = new InstantCommand(() -> setOutake());
        return command;
    }
    
    @Override
    public void periodic(){
        checkCoralLoaded();
        coralLoadedTrigger();
        
        if(coralLoaded && isLoading){
            shooterZeroSpeedCommand();
        }

        SmartDashboard.putNumber("Current Draw", getCurrentDraw());
        SmartDashboard.putBoolean("Coral Loaded", coralLoaded);
    }
}