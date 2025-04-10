package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;

public class PaulServo extends SubsystemBase{

    Servo paulServo = new Servo(1);
    
    public PaulServo(){
        setDisengage();
    }
    
    public void setDisengage() {
        paulServo.set(.2);
    }

    public void setEngage() {
        paulServo.set(0);
    }

    public Command setDisengageCommand() {
        return new InstantCommand(this::setDisengage);
    }
    public Command setEngageCommand() {
        return new InstantCommand(this::setEngage);
    }
}
