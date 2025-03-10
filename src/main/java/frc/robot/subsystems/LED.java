package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class LED extends SubsystemBase {
  
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer tempBuffer; // For power calculations

    // LED patterns
    private final LEDPattern solidRed;
    private final LEDPattern solidBlue;
    private final LEDPattern solidOrange;
    private final LEDPattern blueFlame;
    private final LEDPattern redFlame;
    private final LEDPattern blueBreathing;
    private final LEDPattern redBreathing;
    private final LEDPattern blueBeam;
    private final LEDPattern redBeam;
    
    private LEDPattern currentPattern;
    
    private double currentBrightness = LEDConstants.DEFAULT_BRIGHTNESS;

    // Define our own LEDPattern interface since it's not standard in WPILib
    private interface LEDPattern {
        void applyTo(AddressableLEDBuffer buffer);
        
        // Helper method to create solid color pattern
        static LEDPattern solid(Color color) {
            return buffer -> {
                // Apply current dynamic brightness to solid colors
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, color);
                }
            };
        }
    }

    public LED() {
      led = new AddressableLED(LEDConstants.port);
      buffer = new AddressableLEDBuffer(LEDConstants.length);
      tempBuffer = new AddressableLEDBuffer(LEDConstants.length); // For calculations
      led.setLength(LEDConstants.length);
      led.start();
      
      solidOrange = LEDPattern.solid(Color.kOrange);
      // Create solid alliance color patterns
      solidRed = LEDPattern.solid(Color.kRed);
      solidBlue = LEDPattern.solid(Color.kBlue);

      // Create flame patterns (keeping for future use)
      blueFlame = createFlamePattern(Color.kBlue);
      redFlame = createFlamePattern(Color.kRed);
      
      // Create breathing patterns
      blueBreathing = createBreathingPattern(Color.kBlue);
      redBreathing = createBreathingPattern(Color.kRed);
      
      // Create beam patterns
      blueBeam = createBeamPattern(Color.kBlue);
      redBeam = createBeamPattern(Color.kRed);
      
      currentPattern = solidOrange;
    }
  
    @Override
    public void periodic() {
      // Check if robot is disabled
      boolean currentlyDisabled = DriverStation.isDisabled();
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
      
      if (currentlyDisabled) {
        // Robot is disabled - use breathing patterns instead of flames for now
        currentPattern = (alliance == Alliance.Blue) ? blueBreathing : redBreathing;
      } else {
        // Robot is enabled - use solid alliance colors to save CPU
        currentPattern = (alliance == Alliance.Blue) ? solidBlue : solidRed;
      }

      // Apply the current pattern to temporary buffer for power calculation
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
      // Apply pattern to temp buffer first
      currentPattern.applyTo(tempBuffer);
      
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
     * Creates a breathing pattern in the specified color
     * 
     * @param baseColor The base color for the breathing pattern
     * @return A breathing pattern LEDPattern
     */
    private LEDPattern createBreathingPattern(Color baseColor) {
      return new LEDPattern() {
        private final Timer breathingTimer = new Timer();
        private boolean initialized = false;
        
        {
          // Initialize timer
          breathingTimer.start();
        }
        
        @Override
        public void applyTo(AddressableLEDBuffer buffer) {
          if (!initialized) {
            breathingTimer.reset();
            initialized = true;
          }
          
          // Calculate the phase of the breathing cycle (0 to 1)
          double time = breathingTimer.get();
          double cyclePosition = (time % LEDConstants.BREATHING_CYCLE_PERIOD) / LEDConstants.BREATHING_CYCLE_PERIOD;
          
          // Calculate brightness using a sine wave for smooth transitions
          // sin function oscillates between -1 and 1, so we adjust to 0-1 range
          double breathIntensity = (Math.sin(2 * Math.PI * cyclePosition) + 1) / 2;
          
          // Map to our desired min/max range
          breathIntensity = LEDConstants.BREATHING_MIN_INTENSITY + 
              breathIntensity * (LEDConstants.BREATHING_MAX_INTENSITY - LEDConstants.BREATHING_MIN_INTENSITY);
          
          // Apply the breathing effect to each LED
          for (int i = 0; i < buffer.getLength(); i++) {
            Color breathColor = new Color(
                baseColor.red * breathIntensity,
                baseColor.green * breathIntensity,
                baseColor.blue * breathIntensity
            );
            buffer.setLED(i, breathColor);
          }
        }
      };
    }
    
    // Existing adjustBrightnessForPower and applyPatternWithBrightness methods...
    
    /**
     * Creates a flame pattern in the specified color
     * (Keeping this code intact for future use)
     * 
     * @param baseColor The base color for the flame pattern
     * @return A flame pattern LEDPattern
     */
    private LEDPattern createFlamePattern(Color baseColor) {
      return new LEDPattern() {
        private int cycle = 0;
        private final double[] flameIntensity = new double[LEDConstants.TOTAL_LEDS];
        private final double[] targetIntensity = new double[LEDConstants.TOTAL_LEDS];
        private final double[] changeSpeed = new double[LEDConstants.TOTAL_LEDS];
        
        // Initialize random flame behavior
        {
            for (int i = 0; i < LEDConstants.TOTAL_LEDS; i++) {
                flameIntensity[i] = Math.random() * 
                    (LEDConstants.MAX_FLAME_INTENSITY - LEDConstants.MIN_FLAME_INTENSITY) + 
                    LEDConstants.MIN_FLAME_INTENSITY;
                targetIntensity[i] = Math.random() * 
                    (LEDConstants.MAX_TARGET_INTENSITY - LEDConstants.MIN_TARGET_INTENSITY) + 
                    LEDConstants.MIN_TARGET_INTENSITY;
                changeSpeed[i] = Math.random() * 
                    (LEDConstants.MAX_CHANGE_SPEED - LEDConstants.MIN_CHANGE_SPEED) + 
                    LEDConstants.MIN_CHANGE_SPEED;
            }
        }
        
        @Override
        public void applyTo(AddressableLEDBuffer buffer) {
            cycle = (cycle + 1) % LEDConstants.ANIMATION_CYCLE_MODULO;
            
            // Update flame intensities for a more natural flickering
            if (cycle % LEDConstants.FLAME_UPDATE_INTERVAL == 0) {
                for (int i = 0; i < LEDConstants.TOTAL_LEDS; i++) {
                    // Occasionally set new target intensity
                    if (Math.random() < LEDConstants.NEW_TARGET_PROBABILITY) {
                        targetIntensity[i] = Math.random() * 
                            (LEDConstants.MAX_TARGET_INTENSITY - LEDConstants.MIN_TARGET_INTENSITY) + 
                            LEDConstants.MIN_TARGET_INTENSITY;
                    }
                    
                    // Move current intensity toward target
                    if (flameIntensity[i] < targetIntensity[i]) {
                        flameIntensity[i] = Math.min(flameIntensity[i] + changeSpeed[i], targetIntensity[i]);
                    } else {
                        flameIntensity[i] = Math.max(flameIntensity[i] - changeSpeed[i], targetIntensity[i]);
                    }
                }
            }
            
            // Apply the effect to each LED, taking into account its position in the layout
            for (int i = 0; i < buffer.getLength(); i++) {
                // Determine which section this LED is in (left side, top, or right side)
                double baseIntensity = flameIntensity[i];
                double brightness;
                
                if (i < LEDConstants.LEFT_LEDS) {
                    // Left side - flames rise from bottom to top
                    brightness = baseIntensity * (LEDConstants.LEFT_SINE_OFFSET + LEDConstants.LEFT_SINE_AMPLITUDE * 
                                Math.sin((i + cycle) / LEDConstants.LEFT_SINE_FREQUENCY));
                    // Higher intensity at the bottom
                    brightness *= LEDConstants.LEFT_BOTTOM_OFFSET + LEDConstants.LEFT_BOTTOM_INTENSITY * 
                                (1.0 - (double)i / LEDConstants.LEFT_LEDS);
                } 
                else if (i < LEDConstants.LEFT_LEDS + LEDConstants.TOP_LEDS) {
                    // Top bar - flames spread from sides to middle
                    int topPos = i - LEDConstants.LEFT_LEDS;
                    brightness = baseIntensity * (LEDConstants.TOP_COSINE_OFFSET + LEDConstants.TOP_COSINE_AMPLITUDE * 
                                Math.cos((topPos + cycle) / LEDConstants.TOP_COSINE_FREQUENCY));
                    // More intense at edges, less in middle
                    brightness *= LEDConstants.TOP_EDGE_OFFSET + LEDConstants.TOP_EDGE_INTENSITY * 
                                Math.min((double)topPos / (LEDConstants.TOP_LEDS/2), 
                                (double)(LEDConstants.TOP_LEDS - topPos) / (LEDConstants.TOP_LEDS/2));
                } 
                else {
                    // Right side - flames rise from bottom to top
                    int rightPos = i - LEDConstants.LEFT_LEDS - LEDConstants.TOP_LEDS;
                    brightness = baseIntensity * (LEDConstants.RIGHT_SINE_OFFSET + LEDConstants.RIGHT_SINE_AMPLITUDE * 
                                Math.sin((rightPos + cycle) / LEDConstants.RIGHT_SINE_FREQUENCY));
                    // Higher intensity at the bottom
                    brightness *= LEDConstants.RIGHT_BOTTOM_OFFSET + LEDConstants.RIGHT_BOTTOM_INTENSITY * 
                                (1.0 - (double)rightPos / LEDConstants.RIGHT_LEDS);
                }
                
                // Constrain brightness and apply MAX_BRIGHTNESS
                brightness = Math.min(
                    Math.max(brightness, LEDConstants.MIN_BRIGHTNESS_LIMIT), 
                    LEDConstants.MAX_BRIGHTNESS_LIMIT
                );
                
                // Add some color variation based on intensity (hotter flame is more yellow/white)
                Color flameColor;
                
                if (baseColor == Color.kRed) {
                    flameColor = new Color(
                        baseColor.red * brightness,
                        baseColor.green * brightness * (LEDConstants.RED_GREEN_MULTIPLIER_BASE + 
                                LEDConstants.RED_GREEN_MULTIPLIER_SCALE * brightness),
                        baseColor.blue * brightness * LEDConstants.BLUE_RED_BRIGHTNESS_MULTIPLIER * brightness
                    );
                } else {
                    flameColor = new Color(
                        baseColor.red * brightness,
                        baseColor.green * brightness * LEDConstants.BLUE_GREEN_MULTIPLIER,
                        baseColor.blue * brightness
                    );
                }
                
                buffer.setLED(i, flameColor);
            }
        }
      };
    }
  
    /**
     * Creates a beam pattern that runs up both sides and inward from the edges on top
     * 
     * @param baseColor The base color for the beam pattern
     * @return A beam pattern LEDPattern
     */
    private LEDPattern createBeamPattern(Color baseColor) {
      return new LEDPattern() {
        private final Timer beamTimer = new Timer();
        private boolean initialized = false;
        
        {
          // Initialize timer
          beamTimer.start();
        }
        
        @Override
        public void applyTo(AddressableLEDBuffer buffer) {
          if (!initialized) {
            beamTimer.reset();
            initialized = true;
          }
          
          // Calculate the time position in the beam cycle (0 to 1)
          double time = beamTimer.get();
          double cyclePosition = (time % LEDConstants.BEAM_CYCLE_TIME) / LEDConstants.BEAM_CYCLE_TIME;
          
          // Clear buffer first (set all LEDs to black/off)
          for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
          }
          
          // Apply beam effect to left side
          applyBeamToSegment(buffer, 
                            0, 
                            LEDConstants.LEFT_LEDS, 
                            cyclePosition, 
                            LEDConstants.LEFT_BEAM_UP, 
                            baseColor);
          
          // Apply beam effect to right side
          applyBeamToSegment(buffer, 
                            LEDConstants.LEFT_LEDS + LEDConstants.TOP_LEDS, 
                            LEDConstants.RIGHT_LEDS, 
                            cyclePosition, 
                            LEDConstants.RIGHT_BEAM_UP, 
                            baseColor);
          
          // Apply beam effect to top, split into left and right halves for inward/outward movement
          int topMiddle = LEDConstants.LEFT_LEDS + (LEDConstants.TOP_LEDS / 2);
          
          // Left half of top (moving right)
          applyBeamToSegment(buffer,
                            LEDConstants.LEFT_LEDS,
                            LEDConstants.TOP_LEDS / 2,
                            cyclePosition,
                            LEDConstants.TOP_BEAM_INWARD,
                            baseColor);
          
          // Right half of top (moving left)
          applyBeamToSegment(buffer,
                            topMiddle,
                            LEDConstants.TOP_LEDS / 2,
                            cyclePosition,
                            !LEDConstants.TOP_BEAM_INWARD, // Inverse for other side
                            baseColor);
        }
        
        /**
         * Apply beam effect to a segment of LEDs
         * 
         * @param buffer The LED buffer to modify
         * @param startIndex Starting index of the segment
         * @param length Length of the segment in LEDs
         * @param cyclePosition Current position in animation cycle (0-1)
         * @param forward Direction of beam motion
         * @param color Base color of beam
         */
        private void applyBeamToSegment(AddressableLEDBuffer buffer, int startIndex, int length, 
                                      double cyclePosition, boolean forward, Color color) {
          
          // Calculate beam position
          double beamPos = cyclePosition * (1 + LEDConstants.BEAM_LENGTH + LEDConstants.BEAM_TRAIL_LENGTH);
          
          // For each LED in the segment
          for (int i = 0; i < length; i++) {
            // Convert LED position to 0-1 range within segment
            double ledPos = (double)i / length;
            
            // Adjust position based on direction
            if (!forward) {
              ledPos = 1.0 - ledPos;
            }
            
            // Calculate distance from beam peak position
            double distFromBeam = Math.abs(ledPos - (beamPos % 1.0));
            
            // Wrap around for continuous effect
            if (distFromBeam > 0.5) {
              distFromBeam = 1.0 - distFromBeam;
            }
            
            // Calculate intensity based on distance from beam
            double intensity;
            if (distFromBeam < LEDConstants.BEAM_LENGTH / 2) {
              // Inside main beam
              intensity = LEDConstants.BEAM_INTENSITY;
            } else if (distFromBeam < LEDConstants.BEAM_LENGTH / 2 + LEDConstants.BEAM_TRAIL_LENGTH) {
              // Inside beam trail
              double trailPos = (distFromBeam - LEDConstants.BEAM_LENGTH / 2) / LEDConstants.BEAM_TRAIL_LENGTH;
              intensity = LEDConstants.BEAM_INTENSITY * (1 - trailPos);
            } else {
              // Outside beam
              intensity = 0;
            }
            
            // Apply the intensity to this LED
            if (intensity > 0) {
              Color beamColor = new Color(
                  color.red * intensity,
                  color.green * intensity,
                  color.blue * intensity
              );
              buffer.setLED(startIndex + i, beamColor);
            }
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