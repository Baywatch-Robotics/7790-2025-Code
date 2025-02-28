package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.ButtonBox;

public class ShooterPivot extends SubsystemBase {

    public float shooterPivotDesiredAngle;

    public boolean isInitialized;

    public boolean isStraight;

    private SparkMax shooterPivotMotor = new SparkMax(ShooterPivotConstants.ID, MotorType.kBrushless);

    private SparkClosedLoopController shooterPivotController = shooterPivotMotor.getClosedLoopController();

    private AbsoluteEncoder shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder();
    
    // Add Scope reference
    public Scope scope;

    public ShooterPivot() {
        shooterPivotMotor.configure(Configs.ShooterPivot.shooterPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        isStraight = true;
        shooterPivotDesiredAngle = (float)shooterPivotEncoder.getPosition();
    }
    
    // New constructor with Scope parameter
    public ShooterPivot(Scope scope) {
        this();
        this.scope = scope;
        // Register this component with the scope system
        if (scope != null) {
            scope.setSubsystems(null); // Pass null for arm as it might not be initialized yet
        }
    }
    
    // Set the scope reference separately (in case constructor injection isn't possible)
    public void setScope(Scope scope) {
        this.scope = scope;
        // Update registration with scope system
        if (scope != null) {
            scope.setSubsystems(null);
        }
    }

    public Scope getScope() {
        return this.scope;
    }

    private void setLeftInitial() {
        shooterPivotDesiredAngle = ShooterPivotConstants.leftAngleInitial;
        isStraight = false;
    }
    private void setRightInitial() {
        shooterPivotDesiredAngle = ShooterPivotConstants.rightAngleInitial;
        isStraight = false;
    }
    private void setCenter() {
        shooterPivotDesiredAngle = ShooterPivotConstants.centerAngle;
        isStraight = true;
    }
    private void setLeftL2() {
        shooterPivotDesiredAngle = ShooterPivotConstants.leftL2Angle;
        isStraight = false;
    }
    private void setRightL2() {
        shooterPivotDesiredAngle = ShooterPivotConstants.rightL2Angle;
        isStraight = false;
    }
    private void setLeftL3() {
        shooterPivotDesiredAngle = ShooterPivotConstants.leftL3Angle;
        isStraight = false;
    }
    private void setRightL3() {
        shooterPivotDesiredAngle = ShooterPivotConstants.rightL3Angle;
        isStraight = false;
    }
    private void setLeftL4() {
        shooterPivotDesiredAngle = ShooterPivotConstants.leftL4Angle;
        isStraight = false;
    }
    private void setRightL4() {
        shooterPivotDesiredAngle = ShooterPivotConstants.rightL4Angle;
        isStraight = false;
    }

    public Command setLeftInitialCommand()
    {
        Command command = new InstantCommand(()-> this.setLeftInitial());
        return command;
    }

    public Command setRightInitalCommand()
    {
        Command command = new InstantCommand(()-> this.setRightInitial());
        return command;
    }

    public Command setCenterCommand()
    {
        Command command = new InstantCommand(()-> this.setCenter());
        return command;
    }

    public Command setLeftL2Command()
    {
        Command command = new InstantCommand(()-> this.setLeftL2());
        return command;
    }

    public Command setRightL2Command()
    {
        Command command = new InstantCommand(()-> this.setRightL2());
        return command;
    }

    public Command setLeftL3Command()
    {
        Command command = new InstantCommand(()-> this.setLeftL3());
        return command;
    }

    public Command setRightL3Command()
    {
        Command command = new InstantCommand(()-> this.setRightL3());
        return command;
    }

    public Command setLeftL4Command()
    {
        Command command = new InstantCommand(()-> this.setLeftL4());
        return command;
    }

    public Command setRightL4Command()
    {
        Command command = new InstantCommand(()-> this.setRightL4());
        return command;
    }

    // Apply vision adjustments if enabled and appropriate
    private void applyVisionAdjustment(int level) {
        // Only apply vision for levels 1-3 or when specifically requested
        if (scope != null && scope.isVisionEnabled() && scope.hasTarget() && level >= 1) {
            float adjustment = (float) scope.calculatePivotAngleAdjustment();
            shooterPivotDesiredAngle += adjustment;
            SmartDashboard.putNumber("Vision Pivot Adjustment", adjustment);
        }
    }

    public Command shooterPivotBasedOnQueueCommand(ButtonBox buttonBox) {

        IntSupplier currentLevelSupplier = buttonBox.currentLevelSupplier;
        BooleanSupplier currentLeftSupplier = buttonBox.currentisLeftSupplier;

        Command command = new InstantCommand(() -> {
            if (currentLevelSupplier != null && currentLeftSupplier != null) {
                int level = currentLevelSupplier.getAsInt();
                
                if (level == 0) {
                    // L1 is always center
                    setCenter();
                } else if (level == 1) {
                    // L2
                    if (currentLeftSupplier.getAsBoolean()) {
                        shooterPivotDesiredAngle = ShooterPivotConstants.leftL2Angle;
                        isStraight = false;
                    } else {
                        shooterPivotDesiredAngle = ShooterPivotConstants.rightL2Angle;
                        isStraight = false;
                    }
                } else if (level == 2) {
                    // L3 - uses same targeting as L2
                    if (currentLeftSupplier.getAsBoolean()) {
                        shooterPivotDesiredAngle = ShooterPivotConstants.leftL3Angle;
                        isStraight = false;
                    } else {
                        shooterPivotDesiredAngle = ShooterPivotConstants.rightL3Angle;
                        isStraight = false;
                    }
                } else if (level == 3) {
                    // L4 - special targeting
                    shooterPivotDesiredAngle = ShooterPivotConstants.centerAngle;
                    isStraight = true;
                }
                
                // Apply vision adjustment after setting base position
                applyVisionAdjustment(level);
            }
        });
        return command;
    }

    public void moveAmount(final double amount) {

        if (Math.abs(amount) < 0.2) {
            return;
        }

        float scale = ShooterPivotConstants.manualMultiplier;

        float f = (float) MathUtil.clamp(shooterPivotDesiredAngle + amount * scale, ShooterPivotConstants.min, ShooterPivotConstants.max);

        shooterPivotDesiredAngle = f;
    }

    @Override
    public void periodic() {

        if (!isInitialized) {
            shooterPivotDesiredAngle = (float)(shooterPivotEncoder.getPosition());
            isInitialized = true;
        }
        
        shooterPivotDesiredAngle = (float)MathUtil.clamp(shooterPivotDesiredAngle, ShooterPivotConstants.min, ShooterPivotConstants.max);
        
        SmartDashboard.putNumber("Shooter Pivot Desired Angle", shooterPivotDesiredAngle);
        SmartDashboard.putNumber("Shooter Pivot Current Angle", shooterPivotEncoder.getPosition());
        
        shooterPivotController.setReference(shooterPivotDesiredAngle, ControlType.kPosition);
    }
}
