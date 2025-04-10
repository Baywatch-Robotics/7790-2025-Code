package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.FunnelConstants;

public class AlgaeArm extends SubsystemBase {

    public double algaeArmDesiredAngle;
    
    private double kDt = 0.02; // 20ms periodic loop time
    
    private boolean isInitialized = false;
    
    // Algae piece detection
    public static boolean algaeLoaded = false;
    private boolean isIntaking = false;
    
    // Debounce timer variables
    private double currentAboveThresholdStartTime = 0;
    private double currentBelowThresholdStartTime = 0;
    private final double DEBOUNCE_TIME = 0.5; // 0.5 seconds for debouncing

    private SparkMax algaeArmMotor = new SparkMax(AlgaeArmConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController algaeArmController = algaeArmMotor.getClosedLoopController();

    private AbsoluteEncoder algaeArmEncoder = algaeArmMotor.getAbsoluteEncoder();
    
    // Add ArmFeedforward controller
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        AlgaeArmConstants.kS, 
        AlgaeArmConstants.kG,
        AlgaeArmConstants.kV,
        AlgaeArmConstants.kA
    );
    
    // Conversion factors to convert between encoder units and radians
    private final double kEncoderToRadians = 2.0 * Math.PI; // Adjust based on encoder's range
    
    // Reference angle for zero position (in radians)
    private final double kHorizontalReferenceRad = Math.PI / 2.0; // 90 degrees, adjust based on setup
    
    // Trapezoidal motion profile objects
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            AlgaeArmConstants.maxVelocity, 
            AlgaeArmConstants.maxAcceleration
        )
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public AlgaeArm() {
        algaeArmMotor.configure(Configs.AlgaeArm.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Converts from encoder units to radians
     */
    private double encoderToRadians(double encoderPosition) {
        return encoderPosition * kEncoderToRadians;
    }
    
    /**
     * Converts from encoder position to radians used by feedforward
     * (typically 0 = horizontal, positive = above horizontal)
     */
    private double encoderToFeedforwardRadians(double encoderPosition) {
        // Convert to radians then adjust for the coordinate system
        return encoderToRadians(encoderPosition) - kHorizontalReferenceRad;
    }

    // Method to get the current draw from the motor
    private double getCurrentDraw() {
        return algaeArmMotor.getOutputCurrent();
    }

    // Method to check if algae is loaded based on current draw with debouncing
    public boolean checkAlgaeLoaded() {
        double currentDraw = getCurrentDraw();
        double currentTime = Timer.getFPGATimestamp();
        
        if (currentDraw > AlgaeArmConstants.currentThreshold) {
            // Reset the "below threshold" timer since we're above threshold
            currentBelowThresholdStartTime = currentTime;
            
            // If this is the first time we're above threshold, start timing
            if (currentAboveThresholdStartTime == 0) {
                currentAboveThresholdStartTime = currentTime;
            }
            
            // Check if we've been above threshold long enough
            if (currentTime - currentAboveThresholdStartTime >= DEBOUNCE_TIME) {
                return algaeLoaded = true;
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
                return algaeLoaded = false;
            }
        }
        
        // Return the current state if we haven't debounced yet
        return algaeLoaded;
    }
    
    public Trigger algaeLoadedTrigger() { 
        return new Trigger(this::checkAlgaeLoaded);
    }

    /**
     * Check if algae arm is in a position that allows funnel to safely extend
     */
    public boolean isSafeForFunnelExtension() {
        // Compare current position with threshold
        // Arm is "down" enough when position value is higher (for this specific setup)
        return algaeArmEncoder.getPosition() >= FunnelConstants.SAFE_ALGAE_ARM_POSITION;
    }

    /**
     * Trigger that activates when the algae arm is in a position that allows
     * the funnel to safely extend fully
     */
    public Trigger safeForFunnelExtensionTrigger() {
        return new Trigger(this::isSafeForFunnelExtension);
    }

    // Define desired positions for the arm
    private void stowUp() {
        algaeArmDesiredAngle = AlgaeArmConstants.stowedUpAngle;
        isIntaking = false;
    }
    
    private void straightOut() {
        algaeArmDesiredAngle = AlgaeArmConstants.straightOutAngle;
        isIntaking = false;
    }
    
    private void groundIntake() {
        algaeArmDesiredAngle = AlgaeArmConstants.groundIntakeAngle;
        isIntaking = true;
    }
    
    private void holdPosition() {
        algaeArmDesiredAngle = AlgaeArmConstants.holdAngle;
        isIntaking = false;
    }

    private void scoreL1Postion() {
        algaeArmDesiredAngle = AlgaeArmConstants.L1ScoreAngle;
        isIntaking = false;
    }
    // Commands to move the arm to the desired positions
    public Command algaeArmStowUpCommand() {
        return new InstantCommand(() -> stowUp());
    }

    public Command algaeArmStraightOutCommand() {
        return new InstantCommand(() -> straightOut());
    }

    public Command algaeArmGroundIntakeCommand() {
        return new InstantCommand(() -> groundIntake());
    }
    
    public Command algaeArmHoldCommand() {
        return new InstantCommand(() -> holdPosition());
    }
    
    public Command algaeArmScoreL1Command() {
        return new InstantCommand(() -> scoreL1Postion());
    }
    
    public void moveAmount(final double amount) {
        if (Math.abs(amount) < 0.2) {
            return;
        }

        double scale = AlgaeArmConstants.manualMultiplier;
        double newAngle = algaeArmDesiredAngle + amount * scale;
        
        algaeArmDesiredAngle = MathUtil.clamp(newAngle, AlgaeArmConstants.min, AlgaeArmConstants.max);
    }

    @Override
    public void periodic() {
        if (!isInitialized) {
            algaeArmDesiredAngle = AlgaeArmConstants.stowedUpAngle;
            m_setpoint = new TrapezoidProfile.State(AlgaeArmConstants.stowedUpAngle, 0);

            
            isInitialized = true;
        }
        
        // Check if algae is loaded
        checkAlgaeLoaded();
        
        // If we have algae and we're in intake position, move to hold position
        if (algaeLoaded && isIntaking) {
            holdPosition();
        }
        
        // Apply position constraints
        algaeArmDesiredAngle = MathUtil.clamp(algaeArmDesiredAngle, AlgaeArmConstants.min, AlgaeArmConstants.max);
        
        // Set goal for motion profile
        m_goal = new TrapezoidProfile.State(algaeArmDesiredAngle, 0);
        
        // Calculate next setpoint
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        
        // Convert positions to radians for feedforward
        double currentPositionRad = encoderToFeedforwardRadians(algaeArmEncoder.getPosition() - AlgaeArmConstants.feedforwardOffset);
        double currentVelocityRad = algaeArmEncoder.getVelocity() * kEncoderToRadians;
        
        // Calculate the feedforward output using radians
        double feedforwardOutput = armFeedforward.calculate(
            currentPositionRad,    // Position in radians (0 = horizontal)
            currentVelocityRad,    // Velocity in radians/second
            0                      // Zero acceleration for now
        );
        
        SmartDashboard.putNumber("Algae Arm Desired Angle", algaeArmDesiredAngle);
        SmartDashboard.putNumber("Algae Arm Current Angle", algaeArmEncoder.getPosition());
        SmartDashboard.putNumber("Algae Arm Current Draw", getCurrentDraw());
        SmartDashboard.putBoolean("Algae Loaded", algaeLoaded);
        SmartDashboard.putBoolean("Safe For Funnel", isSafeForFunnelExtension());
        SmartDashboard.putNumber("Algae Arm Profile Position", m_setpoint.position);
        SmartDashboard.putNumber("Algae Arm Profile Velocity", m_setpoint.velocity);
        SmartDashboard.putNumber("Algae Arm Feedforward", feedforwardOutput);
        SmartDashboard.putNumber("Algae Arm Position (rad)", currentPositionRad);
        SmartDashboard.putNumber("Algae Arm Velocity (rad/s)", currentVelocityRad);
        
        
        SmartDashboard.putNumber("Algae Arm Feedforward", feedforwardOutput);

        // Set motor position using the profile's position and feedforward
        algaeArmController.setReference(
            m_setpoint.position, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0,  // slot 0 for PID
            feedforwardOutput,
            ArbFFUnits.kVoltage
        );
    }
}