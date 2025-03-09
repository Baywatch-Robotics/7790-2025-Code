package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LED extends SubsystemBase {
  
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final LEDPattern red;
    private final LEDPattern blueFlame;
    private final LEDPattern redFlame;
    
    private boolean isDisabled = false;
    private LEDPattern currentPattern;

    // Define our own LEDPattern interface since it's not standard in WPILib
    private interface LEDPattern {
        void applyTo(AddressableLEDBuffer buffer);
        
        // Helper method to create solid color pattern
        static LEDPattern solid(Color color) {
            return buffer -> {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, color);
                }
            };
        }
    }

    public LED() {
      led = new AddressableLED(LEDConstants.port);
      buffer = new AddressableLEDBuffer(LEDConstants.length);
      led.setLength(LEDConstants.length);
      led.start();
      
      red = LEDPattern.solid(Color.kRed);
      blueFlame = createFlamePattern(Color.kBlue);
      redFlame = createFlamePattern(Color.kRed);
      currentPattern = red;

      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      setDefaultCommand(runPattern(red));
    }
  
    @Override
    public void periodic() {
      // Check if robot is disabled
      boolean currentlyDisabled = DriverStation.isDisabled();
      
      // If we just entered disabled mode or are still disabled
      if (currentlyDisabled) {
        if (!isDisabled || currentPattern == red) {
          // We just entered disabled mode or need to update pattern
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
          currentPattern = (alliance == Alliance.Blue) ? blueFlame : redFlame;
        }
      } else {
        // Not disabled, use default pattern
        currentPattern = red;
      }
      
      // Update disabled state
      isDisabled = currentlyDisabled;
      
      // Apply the current pattern
      currentPattern.applyTo(buffer);
      
      // Write the data to the LED strip
      led.setData(buffer);
    }
    
    /**
     * Creates a flame pattern in the specified color
     * 
     * @param baseColor The base color for the flame pattern
     * @return A flame pattern LEDPattern
     */
    private LEDPattern createFlamePattern(Color baseColor) {
      return new LEDPattern() {
        private int cycle = 0;
        
        @Override
        public void applyTo(AddressableLEDBuffer buffer) {
          cycle = (cycle + 1) % 10;
          
          for (int i = 0; i < buffer.getLength(); i++) {
            // Generate a "flame" effect by varying brightness based on position and time
            double brightness = 0.5 + 0.5 * Math.sin((i + cycle) / 2.0);
            
            // Create flame color with brightness variation
            Color flameColor = new Color(
                baseColor.red * brightness,
                baseColor.green * brightness * 0.7,
                baseColor.blue * brightness * 0.8
            );
            
            buffer.setLED(i, flameColor);
          }
        }
      };
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