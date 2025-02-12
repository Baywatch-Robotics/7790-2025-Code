package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Queue;
import java.util.LinkedList;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ButtonBoxConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import java.util.function.DoubleSupplier;

public class ButtonBox extends SubsystemBase {

    private final SwerveSubsystem drivebase;

    public ButtonBox(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Container for joystick input suppliers.
     * Now includes two axes for rotation (simulating an Xbox right stick).
     */
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

    /**
     * Returns JoystickSuppliers for field-oriented drive inputs.
     * 
     * While the target is far (distance > TARGET_PROXIMITY_THRESHOLD), the suppliers
     * output joystick values based on the error between the target position/rotation and
     * the current robot pose.
     * 
     * When the target becomes close (distance <= TARGET_PROXIMITY_THRESHOLD), the target
     * is polled (removed from the queue) and zero inputs are returned (disabling control).
     *
     * @return JoystickSuppliers for x, y, rotationX, and rotationY.
     */
    public JoystickSuppliers getFieldOrientedSuppliers() {
        Pose2d currentPose = drivebase.getPose();
        TargetClass candidate = targetQueue.peek();
        if (candidate == null) {
            return new JoystickSuppliers(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0);
        }
        
        Pose2d candidatePose = candidate.toPose2d();
        double dx = candidatePose.getX() - currentPose.getX();
        double dy = candidatePose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        
        if (distance <= ButtonBoxConstants.allowableError) {
            // Remove the target if close enough.
            getNextTarget();
            return new JoystickSuppliers(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0);
        }
        
        double targetRotationDegrees = candidate.getZ();
        // Inversion flags assumed to be defined in ButtonBoxConstants
        boolean invertX = ButtonBoxConstants.invertX;
        boolean invertY = ButtonBoxConstants.invertY;

        DoubleSupplier xInput = () -> MathUtil.clamp((invertX ? -1 : 1) * (candidatePose.getX() - currentPose.getX()), -1.0, 1.0);
        DoubleSupplier yInput = () -> MathUtil.clamp((invertY ? -1 : 1) * (candidatePose.getY() - currentPose.getY()), -1.0, 1.0);

        DoubleSupplier rotationX = () -> {
            double currentRad = currentPose.getRotation().getRadians();
            double targetRad = Math.toRadians(targetRotationDegrees);
            double diffX = Math.cos(targetRad) - Math.cos(currentRad);
            return MathUtil.clamp(diffX, -1.0, 1.0);
        };

        DoubleSupplier rotationY = () -> {
            double currentRad = currentPose.getRotation().getRadians();
            double targetRad = Math.toRadians(targetRotationDegrees);
            double diffY = Math.sin(targetRad) - Math.sin(currentRad);
            return MathUtil.clamp(diffY, -1.0, 1.0);
        };

        return new JoystickSuppliers(xInput, yInput, rotationX, rotationY);
    }
}