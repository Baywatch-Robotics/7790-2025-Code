package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private SparkMax shooterMotor = new SparkMax(ShooterConstants.ID, MotorType.kBrushless);

    private DigitalInput coralSensor = new DigitalInput(0);

    public static boolean coralLoaded;

    private boolean isLoading;

    public Shooter() {
        shooterMotor.configure(
                Configs.Shooter.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        coralLoaded = false;
        isLoading = false;
    }
               
    

    private void setZeroSpeed() {

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
       // coralLoaded = !coralSensor.get();

      //  if (coralLoaded){
      //      if(!isLoading) {
      //      shooterZeroSpeedCommand();
      //      }
      //  }
    }
}