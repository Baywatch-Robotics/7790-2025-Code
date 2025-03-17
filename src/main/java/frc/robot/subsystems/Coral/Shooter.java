package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    private SparkMax shooterMotor = new SparkMax(ShooterConstants.ID, MotorType.kBrushless);

    public static boolean coralLoaded;

    private boolean isLoading;
    
    // Debounce timer variables
    private double currentAboveThresholdStartTime = 0;
    private double currentBelowThresholdStartTime = 0;
    
    // Reference to RobotContainer for controller rumble
    private RobotContainer robotContainer;

    public Shooter() {
        shooterMotor.configure(
                Configs.Shooter.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        coralLoaded = false;
        isLoading = false;
    }
    
    // Set the RobotContainer reference
    public void setRobotContainer(RobotContainer container) {
        this.robotContainer = container;
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
            if (currentTime - currentAboveThresholdStartTime >= ShooterConstants.DEBOUNCE_TIME) {
                // Check if we're transitioning from not loaded to loaded
                if (!coralLoaded) {
                    // Trigger rumble when coral becomes loaded
                    if (robotContainer != null) {
                        triggerCoralLoadedRumble();
                    }
                }
                coralLoaded = true;
            }
        } else {
            // Reset the "above threshold" timer since we're below threshold
            currentAboveThresholdStartTime = 0;
            
            // If this is the first time we're below threshold, start timing
            if (currentBelowThresholdStartTime == 0) {
                currentBelowThresholdStartTime = currentTime;
            }
            
            // Check if we've been below threshold long enough
            if (currentTime - currentBelowThresholdStartTime >= ShooterConstants.DEBOUNCE_TIME) {
                coralLoaded = false;
            }
        }
        return coralLoaded;
    }
    
    /**
     * Trigger a 3-pulse rumble pattern when coral is loaded
     */
    private void triggerCoralLoadedRumble() {
        // Only proceed if robotContainer is set
        if (robotContainer == null) return;
        
        // Create a command sequence for 3 rumble pulses
        Command rumbleSequence = Commands.sequence(
            // First pulse
            robotContainer.setControllerRumbleCommand(1.0),
            Commands.waitSeconds(0.15),
            robotContainer.stopControllerRumbleCommand(),
            Commands.waitSeconds(0.07),
            
            // Second pulse
            robotContainer.setControllerRumbleCommand(1.0),
            Commands.waitSeconds(0.15),
            robotContainer.stopControllerRumbleCommand(),
            Commands.waitSeconds(0.07),
            
            // Third pulse
            robotContainer.setControllerRumbleCommand(1.0),
            Commands.waitSeconds(0.15),
            robotContainer.stopControllerRumbleCommand()
        );
        
        // Schedule the rumble sequence
        rumbleSequence.schedule();
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
        return new InstantCommand(this::setZeroSpeed, this);
    }

    public Command shooterIntakeCommand() {
        return new InstantCommand(this::setIntake, this);
    }

    public Command shooterOutakeCommand() {
        return new InstantCommand(this::setOutake, this);
    }
    
    @Override
    public void periodic(){
        checkCoralLoaded();
        
        // If coral is loaded and we're still trying to intake, stop the motor
        if(coralLoaded && isLoading){
            setZeroSpeed(); // Directly call method instead of scheduling command
        }

        SmartDashboard.putNumber("Current Draw", getCurrentDraw());
        SmartDashboard.putBoolean("Coral Loaded", coralLoaded);
    }
}