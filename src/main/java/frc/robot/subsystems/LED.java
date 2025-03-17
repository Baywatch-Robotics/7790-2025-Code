package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;

public class LED extends SubsystemBase {
  
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer tempBuffer; // For power calculations

    // LED patterns
    private LEDPattern solidRed;
    private LEDPattern solidBlue;
    private LEDPattern blueBreathing;
    private LEDPattern redBreathing;
    private LEDPattern redBlueGradient;
    
    private LEDPattern currentPattern;
    
    private double currentBrightness = LEDConstants.DEFAULT_BRIGHTNESS;

    public LED() {
      led = new AddressableLED(LEDConstants.port);
      buffer = new AddressableLEDBuffer(LEDConstants.length);
      tempBuffer = new AddressableLEDBuffer(LEDConstants.length); // For calculations
      led.setLength(LEDConstants.length);
      led.start();
      
      // Create solid alliance color patterns
      solidRed = LEDPattern.solid(Color.kRed);
      solidBlue = LEDPattern.solid(Color.kBlue);

      // Create breathing patterns with a period of 1.5 seconds
      blueBreathing = LEDPattern.solid(Color.kBlue).breathe(LEDConstants.BREATHING_CYCLE_PERIOD);
      redBreathing = LEDPattern.solid(Color.kRed).breathe(LEDConstants.BREATHING_CYCLE_PERIOD);
      
      // Create red-blue gradient pattern that shifts over time
      redBlueGradient = LEDPattern.discontinuousGradient(Color.kRed, Color.kBlue)
          .animate(2.0);
      
      // Start with gradient pattern by default until alliance is known
      currentPattern = redBlueGradient;
      
      // Apply initial pattern immediately to ensure LEDs light up even without computer
      currentPattern.applyTo(buffer);
      led.setData(buffer);
    }
  
    @Override
    public void periodic() {
      // Check if robot is disabled
      boolean currentlyDisabled = DriverStation.isDisabled();
      
      // Check if alliance information is available
      var allianceOption = DriverStation.getAlliance();
      
      // Select the appropriate pattern based on conditions
      if (allianceOption.isPresent()) {
        Alliance alliance = allianceOption.get();
        
        if (currentlyDisabled) {
          // Robot is disabled - use breathing patterns
          currentPattern = (alliance == Alliance.Blue) ? blueBreathing : redBreathing;
        } else {
          // Robot is enabled - use solid alliance colors
          currentPattern = (alliance == Alliance.Blue) ? solidBlue : solidRed;
        }
      } else {
        // No alliance information available, use gradient pattern
        currentPattern = redBlueGradient;
      }
      
      // Apply current pattern to temp buffer for power calculations
      currentPattern.applyTo(tempBuffer);
      
      // Calculate power and adjust brightness if needed
      adjustBrightnessForPower();
      
      // Apply the current pattern with adjusted brightness
      applyPatternWithBrightness();
      
      // Write the data to the LED strip
      led.setData(buffer);
    }
    
    /**
     * Calculates current draw based on RGB values and adjusts brightness if needed
     */
    private void adjustBrightnessForPower() {
      double totalCurrentDraw = 0;
      
      // Calculate potential power consumption based on temp buffer
      for (int i = 0; i < tempBuffer.getLength(); i++) {
          double red = tempBuffer.getLED(i).red;
          double green = tempBuffer.getLED(i).green;
          double blue = tempBuffer.getLED(i).blue;
          
          // Calculate current for this LED (in mA)
          double ledCurrent = (red * LEDConstants.MILLIAMPS_PER_RED) +
                              (green * LEDConstants.MILLIAMPS_PER_GREEN) +
                              (blue * LEDConstants.MILLIAMPS_PER_BLUE);
          
          totalCurrentDraw += ledCurrent;
      }
      
      // Convert mA to A
      totalCurrentDraw /= 1000.0;
      
      // Calculate max safe brightness based on current power draw
      if (totalCurrentDraw > 0) {
          // How much we can scale up (or need to scale down) our brightness
          double maxSafeBrightness = (LEDConstants.MAX_AMPERAGE * LEDConstants.POWER_SAFETY_MARGIN) / totalCurrentDraw;
          
          // Gradually adjust brightness for smooth transitions
          if (maxSafeBrightness < currentBrightness) {
              // Need to reduce brightness immediately to stay within power budget
              currentBrightness = maxSafeBrightness;
          } else if (maxSafeBrightness > currentBrightness) {
              // Can increase brightness, but do it gradually
              currentBrightness += (maxSafeBrightness - currentBrightness) * 0.1;
          }
          
          // Ensure brightness is within reasonable limits
          currentBrightness = Math.max(0.05, Math.min(1.0, currentBrightness));
      }
    }
  
    /**
     * Applies the current pattern with adjusted brightness
     */
    private void applyPatternWithBrightness() {
        // No need to apply the pattern again, it's already in tempBuffer
        
        // Copy to main buffer with brightness adjustment
        for (int i = 0; i < buffer.getLength(); i++) {
            Color originalColor = tempBuffer.getLED(i);
            Color adjustedColor = new Color(
                originalColor.red * currentBrightness,
                originalColor.green * currentBrightness,
                originalColor.blue * currentBrightness
            );
            buffer.setLED(i, adjustedColor);
        }
    }
    
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return run(() -> {
        pattern.applyTo(buffer);
        led.setData(buffer);
      });
    }
    
    /**
     * Gets the current estimated power consumption in amps
     * @return Current power consumption in amps
     */
    public double getCurrentPowerConsumption() {
        double totalCurrentDraw = 0;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            Color color = buffer.getLED(i);
            totalCurrentDraw += (color.red * LEDConstants.MILLIAMPS_PER_RED) +
                               (color.green * LEDConstants.MILLIAMPS_PER_GREEN) +
                               (color.blue * LEDConstants.MILLIAMPS_PER_BLUE);
        }
        
        return totalCurrentDraw / 1000.0; // Convert mA to A
    }
}

