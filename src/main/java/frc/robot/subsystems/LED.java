package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final LEDPattern red;

    public LED() {
      led = new AddressableLED(LEDConstants.port);
      buffer = new AddressableLEDBuffer(LEDConstants.length);
      led.setLength(LEDConstants.length);
      led.start();
      
      red = LEDPattern.solid(Color.kRed);

      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      // Note: Other default patterns could be used instead!
      setDefaultCommand(runPattern(red));
    }
  
    @Override
    public void periodic() {

// Write the data to the LED strip
  led.setData(buffer);

    }
  
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return run(() -> pattern.applyTo(buffer));
    }
  }