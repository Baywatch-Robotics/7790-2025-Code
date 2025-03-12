package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDConstants;

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
    private final LEDPattern redBlueGradient; // New gradient pattern
    
    // New patterns based on requirements
    private LEDPattern reefPattern;
    private LEDPattern targetReachedPattern;
    private LEDPattern holdingAlgaePattern;
    private LEDPattern climbingPattern;
    private LEDPattern oneMinLeftPattern;
    private LEDPattern thirtySecondsLeftPattern;
    private LEDPattern countdownPattern;
    private LEDPattern intakePattern;
    
    private LEDPattern currentPattern;
    
    private double currentBrightness = LEDConstants.DEFAULT_BRIGHTNESS;
    
    // Track the current state for pattern selection
    private LEDState currentState = LEDState.DEFAULT;
    
    // Variables for target tracking
    private double targetDistance = Double.MAX_VALUE;
    
    // Variables for countdown
    private double countdownSecondsRemaining = 0;
    private boolean isCountingDown = false;
    
    // Timer for one-shot patterns like match time warnings
    private Timer patternTimer = new Timer();

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
    
    // Enum to track LED states for pattern selection
    public enum LEDState {
        DEFAULT,
        REEF_TRACKING,
        TARGET_REACHED,
        HOLDING_ALGAE,
        CLIMBING,
        ONE_MIN_LEFT,
        THIRTY_SEC_LEFT,
        COUNTDOWN,
        INTAKE
    }

    public LED() {
      led = new AddressableLED(LEDConstants.port);
      buffer = new AddressableLEDBuffer(LEDConstants.length);
      tempBuffer = new AddressableLEDBuffer(LEDConstants.length); // For calculations
      led.setLength(LEDConstants.length);
      led.start();
      
      // Initialize pattern timer
      patternTimer.start();
      
      // Create basic patterns
      solidOrange = LEDPattern.solid(Color.kOrange);
      solidRed = LEDPattern.solid(Color.kRed);
      solidBlue = LEDPattern.solid(Color.kBlue);
      blueFlame = createFlamePattern(Color.kBlue);
      redFlame = createFlamePattern(Color.kRed);
      blueBreathing = createBreathingPattern(Color.kBlue);
      redBreathing = createBreathingPattern(Color.kRed);
      blueBeam = createBeamPattern(Color.kBlue);
      redBeam = createBeamPattern(Color.kRed);
      redBlueGradient = createRedBlueGradientPattern();
      
      // Initialize new patterns
      reefPattern = createReefPattern();
      targetReachedPattern = createTargetReachedPattern();
      holdingAlgaePattern = createHoldingAlgaePattern();
      climbingPattern = createClimbingPattern();
      oneMinLeftPattern = createOneMinLeftPattern();
      thirtySecondsLeftPattern = createThirtySecondsLeftPattern();
      countdownPattern = createCountdownPattern();
      intakePattern = createIntakePattern();
      
      // Start with gradient pattern by default until alliance is known
      currentPattern = redBlueGradient;
      
      // Apply initial pattern immediately to ensure LEDs light up even without computer
      currentPattern.applyTo(buffer);
      led.setData(buffer);
    }
    
    /**
     * Creates a pattern that pulses reef color with frequency based on distance to target
     */
    private LEDPattern createReefPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            {
                timer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                // Convert target distance to pulse frequency (closer = faster pulsing)
                double normalizedDistance = Math.min(targetDistance, LEDConstants.MAX_TARGET_DISTANCE) / LEDConstants.MAX_TARGET_DISTANCE;
                double pulseFrequency = LEDConstants.MIN_PULSE_FREQUENCY + 
                    (1.0 - normalizedDistance) * (LEDConstants.MAX_PULSE_FREQUENCY - LEDConstants.MIN_PULSE_FREQUENCY);
                
                // Calculate pulse intensity using sine wave
                double time = timer.get();
                double pulsePhase = (time * pulseFrequency) % 1.0;
                double pulseIntensity = (Math.sin(pulsePhase * 2 * Math.PI) + 1.0) / 2.0; // Range 0.0-1.0
                
                // Apply the reef color with intensity
                Color reefColor = new Color(
                    LEDConstants.REEF_COLOR[0] * pulseIntensity,
                    LEDConstants.REEF_COLOR[1] * pulseIntensity,
                    LEDConstants.REEF_COLOR[2] * pulseIntensity
                );
                
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, reefColor);
                }
            }
        };
    }
    
    /**
     * Creates a fast flashing green pattern for when target is reached
     */
    private LEDPattern createTargetReachedPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            {
                timer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                double time = timer.get();
                double flashPhase = (time * LEDConstants.TARGET_REACHED_FLASH_FREQUENCY) % 1.0;
                
                // On for half the cycle, off for half
                boolean isOn = flashPhase < 0.5;
                
                Color ledColor = isOn ? 
                    new Color(LEDConstants.GREEN_COLOR[0], LEDConstants.GREEN_COLOR[1], LEDConstants.GREEN_COLOR[2]) : 
                    Color.kBlack;
                
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, ledColor);
                }
            }
        };
    }
    
    /**
     * Creates a solid teal pattern for when holding algae
     */
    private LEDPattern createHoldingAlgaePattern() {
        return buffer -> {
            Color algaeColor = new Color(
                LEDConstants.ALGAE_COLOR[0],
                LEDConstants.ALGAE_COLOR[1],
                LEDConstants.ALGAE_COLOR[2]
            );
            
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, algaeColor);
            }
        };
    }
    
    /**
     * Creates a pattern that transitions from purple to alliance color for climbing
     */
    private LEDPattern createClimbingPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            {
                timer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                double time = timer.get();
                double cycleTime = LEDConstants.CLIMBING_TRANSITION_TIME;
                double phase = (time % cycleTime) / cycleTime;
                
                // Get alliance color
                Color allianceColor;
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    allianceColor = new Color(LEDConstants.RED_COLOR[0], LEDConstants.RED_COLOR[1], LEDConstants.RED_COLOR[2]);
                } else {
                    allianceColor = new Color(LEDConstants.BLUE_COLOR[0], LEDConstants.BLUE_COLOR[1], LEDConstants.BLUE_COLOR[2]);
                }
                
                // Purple color
                Color purpleColor = new Color(LEDConstants.PURPLE_COLOR[0], LEDConstants.PURPLE_COLOR[1], LEDConstants.PURPLE_COLOR[2]);
                
                // Transition from purple to alliance color
                double purpleWeight = Math.cos(phase * Math.PI / 2);  // 1.0 -> 0.0
                double allianceWeight = Math.sin(phase * Math.PI / 2); // 0.0 -> 1.0
                
                Color transitionColor = new Color(
                    purpleColor.red * purpleWeight + allianceColor.red * allianceWeight,
                    purpleColor.green * purpleWeight + allianceColor.green * allianceWeight,
                    purpleColor.blue * purpleWeight + allianceColor.blue * allianceWeight
                );
                
                // Apply a sweeping effect - traveling up both sides
                for (int i = 0; i < buffer.getLength(); i++) {
                    double position;
                    if (i < LEDConstants.LEFT_LEDS) {
                        position = (double)i / LEDConstants.LEFT_LEDS;
                    } else if (i < LEDConstants.LEFT_LEDS + LEDConstants.TOP_LEDS) {
                        position = 1.0; // Top is always full intensity
                    } else {
                        position = (double)(i - LEDConstants.LEFT_LEDS - LEDConstants.TOP_LEDS) / LEDConstants.RIGHT_LEDS;
                    }
                    
                    // Adjust position by time to create movement
                    double adjustedPos = (position + phase) % 1.0;
                    
                    // Create wave intensity - brighter at the wave peak
                    double intensity = 0.7 + 0.3 * Math.max(0, Math.sin(adjustedPos * 2 * Math.PI));
                    
                    Color finalColor = new Color(
                        transitionColor.red * intensity,
                        transitionColor.green * intensity,
                        transitionColor.blue * intensity
                    );
                    
                    buffer.setLED(i, finalColor);
                }
            }
        };
    }
    
    /**
     * Creates a white flashing pattern for 1 minute remaining
     */
    private LEDPattern createOneMinLeftPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            private boolean initialized = false;
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                if (!initialized) {
                    timer.reset();
                    timer.start();
                    initialized = true;
                }
                
                double time = timer.get();
                double flashCycle = time * LEDConstants.TIMER_ALERT_FLASH_FREQUENCY;
                
                // Flash for a specific number of times
                if (flashCycle < LEDConstants.TIMER_ALERT_FLASH_COUNT) {
                    // Each flash cycle is 1.0 unit
                    boolean isOn = (flashCycle % 1.0) < 0.5;
                    
                    Color color = isOn ? 
                        new Color(LEDConstants.WHITE_COLOR[0], LEDConstants.WHITE_COLOR[1], LEDConstants.WHITE_COLOR[2]) : 
                        Color.kBlack;
                    
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setLED(i, color);
                    }
                } else {
                    // After flashing, return to default pattern
                    currentState = LEDState.DEFAULT;
                    
                    // Just fill with black for this frame - next periodic will switch to appropriate pattern
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setLED(i, Color.kBlack);
                    }
                }
            }
        };
    }
    
    /**
     * Creates a yellow flashing pattern for 30 seconds remaining
     */
    private LEDPattern createThirtySecondsLeftPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            private boolean initialized = false;
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                if (!initialized) {
                    timer.reset();
                    timer.start();
                    initialized = true;
                }
                
                double time = timer.get();
                double flashCycle = time * LEDConstants.TIMER_ALERT_FLASH_FREQUENCY;
                
                // Flash for a specific number of times
                if (flashCycle < LEDConstants.TIMER_ALERT_FLASH_COUNT) {
                    // Each flash cycle is 1.0 unit
                    boolean isOn = (flashCycle % 1.0) < 0.5;
                    
                    Color color = isOn ? 
                        new Color(LEDConstants.YELLOW_COLOR[0], LEDConstants.YELLOW_COLOR[1], LEDConstants.YELLOW_COLOR[2]) : 
                        Color.kBlack;
                    
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setLED(i, color);
                    }
                } else {
                    // After flashing, return to default pattern
                    currentState = LEDState.DEFAULT;
                    
                    // Just fill with black for this frame - next periodic will switch to appropriate pattern
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setLED(i, Color.kBlack);
                    }
                }
            }
        };
    }
    
    /**
     * Creates a countdown pattern with the opposite alliance color
     */
    private LEDPattern createCountdownPattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            {
                timer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                if (!isCountingDown) {
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setLED(i, Color.kBlack);
                    }
                    return;
                }
                
                // Get opposite alliance color
                Color oppositeColor;
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    oppositeColor = new Color(LEDConstants.BLUE_COLOR[0], LEDConstants.BLUE_COLOR[1], LEDConstants.BLUE_COLOR[2]);
                } else {
                    oppositeColor = new Color(LEDConstants.RED_COLOR[0], LEDConstants.RED_COLOR[1], LEDConstants.RED_COLOR[2]);
                }
                
                // Calculate flash frequency based on time remaining (gets faster as time runs out)
                double baseFrequency = LEDConstants.COUNTDOWN_FLASH_FREQUENCY;
                double frequencyMultiplier = Math.max(1.0, 10.0 - countdownSecondsRemaining);
                double flashFrequency = baseFrequency * frequencyMultiplier;
                
                // Calculate flash state
                double time = timer.get();
                boolean isOn = (time * flashFrequency % 1.0) < 0.5;
                
                // Apply color to all LEDs
                Color color = isOn ? oppositeColor : Color.kBlack;
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, color);
                }
            }
        };
    }
    
    /**
     * Creates a pattern for when intake is running
     */
    private LEDPattern createIntakePattern() {
        return new LEDPattern() {
            private Timer timer = new Timer();
            {
                timer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
                double time = timer.get();
                double cyclePosition = (time % LEDConstants.INTAKE_PATTERN_CYCLE_TIME) / LEDConstants.INTAKE_PATTERN_CYCLE_TIME;
                
                // Get alliance color
                Color allianceColor;
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    allianceColor = new Color(LEDConstants.RED_COLOR[0], LEDConstants.RED_COLOR[1], LEDConstants.RED_COLOR[2]);
                } else {
                    allianceColor = new Color(LEDConstants.BLUE_COLOR[0], LEDConstants.BLUE_COLOR[1], LEDConstants.BLUE_COLOR[2]);
                }
                
                // Create a "chasing" light effect from top to bottom
                for (int i = 0; i < buffer.getLength(); i++) {
                    double position;
                    if (i < LEDConstants.LEFT_LEDS) {
                        position = 1.0 - ((double)i / LEDConstants.LEFT_LEDS); // 1.0->0.0 top to bottom on left
                    } else if (i < LEDConstants.LEFT_LEDS + LEDConstants.TOP_LEDS) {
                        position = 1.0; // Top section is always lit
                    } else {
                        position = 1.0 - ((double)(i - LEDConstants.LEFT_LEDS - LEDConstants.TOP_LEDS) / LEDConstants.RIGHT_LEDS); 
                        // 1.0->0.0 top to bottom on right
                    }
                    
                    // Create a moving segment effect
                    double segment = (position - cyclePosition) * 3.0 % 1.0; // segments are 1/3 of the strip
                    double intensity = Math.max(0, 1.0 - Math.abs(segment - 0.5) * 4.0); // peak at 0.5, zero elsewhere
                    
                    // Mix between alliance color and white based on intensity
                    Color segmentColor = new Color(
                        allianceColor.red * (1 - intensity) + intensity,
                        allianceColor.green * (1 - intensity) + intensity,
                        allianceColor.blue * (1 - intensity) + intensity
                    );
                    
                    buffer.setLED(i, segmentColor);
                }
            }
        };
    }
  
    @Override
    public void periodic() {
      // Select pattern based on current state and conditions
      selectPattern();
      
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
     * Selects the appropriate pattern based on current state
     */
    private void selectPattern() {
        // Check for match time alerts
        double matchTime = DriverStation.getMatchTime();
        if (DriverStation.isTeleop() && matchTime <= 60.0 && matchTime >= 59.5 && currentState != LEDState.ONE_MIN_LEFT) {
            setLEDState(LEDState.ONE_MIN_LEFT);
        } else if (DriverStation.isTeleop() && matchTime <= 30.0 && matchTime >= 29.5 && currentState != LEDState.THIRTY_SEC_LEFT) {
            setLEDState(LEDState.THIRTY_SEC_LEFT);
        } else if (DriverStation.isTeleop() && matchTime <= 10.0 && currentState != LEDState.COUNTDOWN) {
            setLEDState(LEDState.COUNTDOWN);
            countdownSecondsRemaining = matchTime;
            isCountingDown = true;
        } else if (isCountingDown) {
            countdownSecondsRemaining = matchTime;
            if (matchTime <= 0) {
                isCountingDown = false;
                setLEDState(LEDState.DEFAULT);
            }
        }
        
        // Select pattern based on current state
        switch (currentState) {
            case REEF_TRACKING:
                currentPattern = reefPattern;
                break;
                
            case TARGET_REACHED:
                currentPattern = targetReachedPattern;
                break;
                
            case HOLDING_ALGAE:
                currentPattern = holdingAlgaePattern;
                break;
                
            case CLIMBING:
                currentPattern = climbingPattern;
                break;
                
            case ONE_MIN_LEFT:
                currentPattern = oneMinLeftPattern;
                break;
                
            case THIRTY_SEC_LEFT:
                currentPattern = thirtySecondsLeftPattern;
                break;
                
            case COUNTDOWN:
                currentPattern = countdownPattern;
                break;
                
            case INTAKE:
                currentPattern = intakePattern;
                break;
                
            case DEFAULT:
            default:
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
                break;
        }
    }
    
    /**
     * Sets the LED state and resets pattern timer
     */
    public void setLEDState(LEDState state) {
        currentState = state;
        patternTimer.reset();
        patternTimer.start();
    }
    
    /**
     * Updates the target distance for reef pattern
     */
    public void setTargetDistance(double distance) {
        this.targetDistance = distance;
    }
    
    // Command generators for different LED patterns
    
    /**
     * Command to show reef tracking pattern with pulsing based on distance
     * @param distance The distance to target in meters
     */
    public Command reefTrackingCommand(double distance) {
        return new InstantCommand(() -> {
            setTargetDistance(distance);
            setLEDState(LEDState.REEF_TRACKING);
        });
    }
    
    /**
     * Command to show target reached pattern (fast flashing green)
     */
    public Command targetReachedCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.TARGET_REACHED));
    }
    
    /**
     * Command to show holding algae pattern (solid teal)
     */
    public Command holdingAlgaeCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.HOLDING_ALGAE));
    }
    
    /**
     * Command to show climbing pattern (purple to alliance color animation)
     */
    public Command climbingCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.CLIMBING));
    }
    
    /**
     * Command to run the intake pattern 
     */
    public Command intakeCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.INTAKE));
    }
    
    /**
     * Command to revert to default pattern
     */
    public Command defaultCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.DEFAULT));
    }
    
    /**
     * Command to run a pattern for a specific duration then revert to default
     */
    public Command timedPatternCommand(LEDState state, double durationSeconds) {
        return new InstantCommand(() -> setLEDState(state))
            .andThen(new WaitCommand(durationSeconds))
            .andThen(new InstantCommand(() -> setLEDState(LEDState.DEFAULT)));
    }
    
    /**
     * Command to flash white for 1 minute warning
     */
    public Command oneMinuteWarningCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.ONE_MIN_LEFT));
    }
    
    /**
     * Command to flash yellow for 30 seconds warning
     */
    public Command thirtySecondWarningCommand() {
        return new InstantCommand(() -> setLEDState(LEDState.THIRTY_SEC_LEFT));
    }
    
    /**
     * Command to start countdown pattern
     */
    public Command startCountdownCommand(double seconds) {
        return new InstantCommand(() -> {
            countdownSecondsRemaining = seconds;
            isCountingDown = true;
            setLEDState(LEDState.COUNTDOWN);
        });
    }
    
    /**
     * Creates a gradient pattern that transitions from red to blue
     */
    private LEDPattern createRedBlueGradientPattern() {
        // ... existing implementation ...
        return new LEDPattern() {
            private final Timer animationTimer = new Timer();
            
            {
              // Initialize timer immediately
              animationTimer.start();
            }
            
            @Override
            public void applyTo(AddressableLEDBuffer buffer) {
              // Make the gradient shift over time for a dynamic effect
              double time = animationTimer.get() * 0.5; // Controls the speed of the animation
              double cycle = (time % 1.0);
              
              int length = buffer.getLength();
              
              // Generate gradient
              for (int i = 0; i < length; i++) {
                // Calculate position in the gradient (0 to 1)
                double position = ((double)i / length + cycle) % 1.0;
                
                // Create color based on position
                // First half transitions from red to purple
                // Second half transitions from purple to blue
                double red, blue;
                
                if (position < 0.5) {
                    // 0.0 to 0.5: Red to Purple
                    red = 1.0;
                    blue = position * 2.0; // 0.0 to 1.0
                } else {
                    // 0.5 to 1.0: Purple to Blue
                    red = 1.0 - ((position - 0.5) * 2.0); // 1.0 to 0.0
                    blue = 1.0;
                }
                
                // Apply a pulse/wave effect
                double brightness = 0.5 + 0.5 * Math.sin(time * 2 * Math.PI + i * Math.PI / 20);
                
                // Create the color with gradient and wave effect
                Color gradientColor = new Color(red * brightness, 0, blue * brightness);
                buffer.setLED(i, gradientColor);
              }
            }
          };
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