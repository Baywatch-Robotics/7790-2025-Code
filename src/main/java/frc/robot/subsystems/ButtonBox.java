package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Queue;
import java.util.LinkedList;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.TargetClassConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ButtonBox extends SubsystemBase {

    private Queue<TargetClass> targetQueue = new LinkedList<>();
    private TargetClass lastAddedTarget = null; // Store the last target that was added
    private final SwerveSubsystem swerveSubsystem;

    public ButtonBox(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void addTarget(TargetClass target) {
        targetQueue.add(target);
        lastAddedTarget = target; // Save reference to last added target
        updateDashboard();
    }

    public void addTarget(String targetName) {
        TargetClass target = TargetClass.GetTargetByName(targetName);
        addTarget(target); // Use the other method to ensure lastAddedTarget is set
    }

    public void clearTargets() {
        targetQueue.clear();
        // Clear target visualization on the field
        if (swerveSubsystem != null) {
            swerveSubsystem.clearTargetVisualization();
        }
        updateDashboard();
    }
    
    public IntSupplier currentLevelSupplier = () -> {
        var target = targetQueue.peek();
        return target != null ? target.getLevel() : 0;
    };
    public IntSupplier currentFaceSupplier = () -> {
        var target = targetQueue.peek();
        return target != null ? target.getFace() : 0;
    };
    public BooleanSupplier currentisLeftSupplier = () -> {
        var target = targetQueue.peek();
        return target != null && target.isLeft();
    };
    
    public Supplier<TargetClass> currentTargetClassSupplier = () -> targetQueue.peek();
    
    // Suppliers for targets ending in '1' (for Right commands)
    public IntSupplier currentLevelSupplierEndingIn1 = () -> {
        var target = peekNextTargetEndingIn1();
        return target != null ? target.getLevel() : 0;
    };
    public IntSupplier currentFaceSupplierEndingIn1 = () -> {
        var target = peekNextTargetEndingIn1();
        return target != null ? target.getFace() : 0;
    };
    public BooleanSupplier currentisLeftSupplierEndingIn1 = () -> {
        var target = peekNextTargetEndingIn1();
        return target != null && target.isLeft();
    };
    
    public Supplier<TargetClass> currentTargetClassSupplierEndingIn1 = () -> peekNextTargetEndingIn1();
    
    // Suppliers for targets ending in '0' (for Left commands)
    public IntSupplier currentLevelSupplierEndingIn0 = () -> {
        var target = peekNextTargetEndingIn0();
        return target != null ? target.getLevel() : 0;
    };
    public IntSupplier currentFaceSupplierEndingIn0 = () -> {
        var target = peekNextTargetEndingIn0();
        return target != null ? target.getFace() : 0;
    };
    public BooleanSupplier currentisLeftSupplierEndingIn0 = () -> {
        var target = peekNextTargetEndingIn0();
        return target != null && target.isLeft();
    };
    
    public Supplier<TargetClass> currentTargetClassSupplierEndingIn0 = () -> peekNextTargetEndingIn0();
    
    public void deleteLastTarget() {

        if(targetQueue.isEmpty()) {
            return;
        }
        if (targetQueue instanceof LinkedList) {
            ((LinkedList<TargetClass>) targetQueue).removeLast();
        }
        updateDashboard();
    }

    public void deleteFirstTarget() {
        targetQueue.poll();
        updateDashboard();
    }

    public TargetClass getNextTarget() {
        TargetClass target = targetQueue.poll();
        updateDashboard();
        return target;
    }

    public TargetClass peekNextTargetEndingIn1() {
        // Search through the queue for the first target ending in '1' without removing anything
        for (TargetClass target : targetQueue) {
            if (target.getName().endsWith("1")) {
                return target;
            }
        }
        return null;
    }

    public TargetClass peekNextTargetEndingIn0() {
        // Search through the queue for the first target ending in '0' without removing anything
        for (TargetClass target : targetQueue) {
            if (target.getName().endsWith("0")) {
                return target;
            }
        }
        return null;
    }

    public TargetClass peekNextTarget() {
        return targetQueue.peek();

    }
    
    
    public Command getNextTargetCommand() {
        return new InstantCommand(() -> getNextTarget());
    }
    public Command peekNextTargetCommand() {
        return new InstantCommand(() -> peekNextTarget());
    }

    public String[] getQueueString() {
        String[] arr = new String[targetQueue.size()];
        int i = 0;
        for (TargetClass t : targetQueue) {
            arr[i++] = t.toString();
        }
        return arr;
    }

    public void updateDashboard() {
        String[] queueArray = getQueueString();
        String queueString = String.join(", ", queueArray);
        SmartDashboard.putString("Target List", queueString);
    }

    public boolean hasQueue() {
        return !targetQueue.isEmpty();
    }

    public void requeueLastTarget() {
        if (lastAddedTarget != null) {
            targetQueue.add(lastAddedTarget);
            updateDashboard();
        }
    }
    
    public Command requeueLastTargetCommand() {
        return new InstantCommand(() -> requeueLastTarget());
    }

    public boolean hasLastTarget() {
        return lastAddedTarget != null;
    }
    
    public BooleanSupplier isHighAlgaeSupplier = () -> {
        var target = targetQueue.peek();
        if (target == null) return false;
        
        String name = target.getName();
        if (!name.startsWith("A")) return false;
        
        int face = target.getFace();
        // Based on the constants in TargetClassConstants that define which faces have high algae
        switch (face) {
            case 1: return TargetClassConstants.isHighAlgaeA1XX;
            case 2: return TargetClassConstants.isHighAlgaeA2XX;
            case 3: return TargetClassConstants.isHighAlgaeA3XX;
            case 4: return TargetClassConstants.isHighAlgaeA4XX;
            case 5: return TargetClassConstants.isHighAlgaeA5XX;
            case 6: return TargetClassConstants.isHighAlgaeA6XX;
            default: return false;
        }
    };
    
    // Check if current target is an algae target
    public BooleanSupplier isAlgaeTargetSupplier = () -> {
        var target = targetQueue.peek();
        return target != null && target.getName().startsWith("A");
    };
    
    // Direct methods to check ball properties without using suppliers
    public boolean isCurrentTargetBall() {
        TargetClass target = peekNextTarget();
        return target != null && target.getName().startsWith("A");
    }

    public boolean isCurrentBallHigh() {
        TargetClass target = peekNextTarget();
        if (target == null || !target.getName().startsWith("A")) {
            return false;
        }
        
        int face = target.getFace();
        // Check if it's a high ball based on face
        switch (face) {
            case 1: return TargetClassConstants.isHighAlgaeA1XX;
            case 2: return TargetClassConstants.isHighAlgaeA2XX;
            case 3: return TargetClassConstants.isHighAlgaeA3XX;
            case 4: return TargetClassConstants.isHighAlgaeA4XX;
            case 5: return TargetClassConstants.isHighAlgaeA5XX;
            case 6: return TargetClassConstants.isHighAlgaeA6XX;
            default: return false;
        }
    }

    // Command to set elevator based on current ball target
    public Command setElevatorForCurrentBallCommand(Elevator elevator) {
        return new InstantCommand(() -> {
            if (isCurrentTargetBall()) {
                if (isCurrentBallHigh()) {
                    elevator.setHighBall();
                } else {
                    elevator.setLowBall();
                }
            }
        });
    }
    
    /*
    public static class JoystickSuppliers {
        public final DoubleSupplier x;
        public final DoubleSupplier y;
        public final DoubleSupplier rotationX;
        public final DoubleSupplier rotationY;

        public JoystickSuppliers(DoubleSupplier x, DoubleSupplier y,
                                 DoubleSupplier rotationX, DoubleSupplier rotationY) {
            this.x = x;
            this.y = y;
            this.rotationX = rotationX;
            this.rotationY = rotationY;
        }
    }
    

    public JoystickSuppliers getFieldOrientedSuppliers() {
        Pose2d currentPose = drivebase.getPose();
        TargetClass candidate = targetQueue.peek();
        if (candidate == null) {
            return new JoystickSuppliers(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0);
        }
        
        Pose2d targetPose = new Pose2d(candidate.getX(), candidate.getY(), new Rotation2d(candidate.getZ()));

        Pose2d candidatePose = candidate.toPose2d(targetPose);
        double dx = candidatePose.getX() - currentPose.getX();
        double dy = candidatePose.getY() - currentPose.getY();
        distance = Math.hypot(dx, dy);
        
        if (distance <= ButtonBoxConstants.allowableError) {
            // Remove the target if close enough.
            return new JoystickSuppliers(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0);
        }
        
        
        double targetRad = candidate.getZ();
        // Inversion flags assumed to be defined in ButtonBoxConstants
        
        var alliance = DriverStation.getAlliance();
        
        final boolean invertX = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        final boolean invertY = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        DoubleSupplier xInput;
        DoubleSupplier yInput;
        
        if (distance >= ButtonBoxConstants.fastMoveThreshold) {
            xInput = () -> MathUtil.clamp(ButtonBoxConstants.p * (invertX ? -1 : 1) * (candidatePose.getX() - currentPose.getX()), ButtonBoxConstants.lowClamp, ButtonBoxConstants.highClamp);
            yInput = () -> MathUtil.clamp(ButtonBoxConstants.p * (invertY ? -1 : 1) * (candidatePose.getY() - currentPose.getY()), ButtonBoxConstants.lowClamp, ButtonBoxConstants.highClamp);
        }
        else if (!elevator.isAtSetpointBoolean()) {
            //xInput = () -> MathUtil.clamp(ButtonBoxConstants.pSuperSlow * (invertX ? -1 : 1) * (candidatePose.getX() - currentPose.getX()), ButtonBoxConstants.lowClampSuperSlowX, ButtonBoxConstants.highClampSuperSlowX);
            //yInput = () -> MathUtil.clamp(ButtonBoxConstants.pSuperSlow * (invertY ? -1 : 1) * (candidatePose.getY() - currentPose.getY()), ButtonBoxConstants.lowClampSuperSlowY, ButtonBoxConstants.highClampSuperSlowY);
            xInput = () -> 0.0;
            yInput = () -> 0.0;
        }
        else {
            xInput = () -> MathUtil.clamp(ButtonBoxConstants.pSlow * (invertX ? -1 : 1) * (candidatePose.getX() - currentPose.getX()), ButtonBoxConstants.lowClampSlowX, ButtonBoxConstants.highClampSlowX);
            yInput = () -> MathUtil.clamp(ButtonBoxConstants.pSlow * (invertY ? -1 : 1) * (candidatePose.getY() - currentPose.getY()), ButtonBoxConstants.lowClampSlowY, ButtonBoxConstants.highClampSlowY);
        }

        // Adjusted rotation calculation by subtracting 90° (i.e. Math.PI/2 radians)
        DoubleSupplier rotationX = () -> MathUtil.clamp(Math.cos(targetRad + Math.PI / 2), -1.0, 1.0);
        DoubleSupplier rotationY = () -> MathUtil.clamp(Math.sin(targetRad + Math.PI / 2), -1.0, 1.0);

        SmartDashboard.putNumber("xInput", xInput.getAsDouble());
        SmartDashboard.putNumber("yInput", yInput.getAsDouble());
        SmartDashboard.putNumber("rotationX", rotationX.getAsDouble());
        SmartDashboard.putNumber("rotationY", rotationY.getAsDouble());
        SmartDashboard.putNumber("distance", distance);


        return new JoystickSuppliers(xInput, yInput, rotationX, rotationY);
    }
        */
}