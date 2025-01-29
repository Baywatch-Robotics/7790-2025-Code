package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    AddressableLED ledSystem = new AddressableLED(0);

    public LED() {
        
    }
}
