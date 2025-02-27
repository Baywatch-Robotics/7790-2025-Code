package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Queue;
import java.util.LinkedList;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class ButtonBox extends SubsystemBase {

    private Queue<TargetClass> targetQueue = new LinkedList<>();

    public void addTarget(TargetClass target) {
        targetQueue.add(target);
        updateDashboard();
    }

    public void addTarget(String targetName) {
        addTarget(TargetClass.GetTargetByName(targetName));
    }

    public void clearTargets() {
        targetQueue.clear();
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

    public void deleteLastTarget() {
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
        SmartDashboard.putStringArray("Target List", getQueueString());
    }

    public boolean hasQueue() {
        return !targetQueue.isEmpty();
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