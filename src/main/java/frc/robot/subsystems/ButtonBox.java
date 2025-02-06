package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Queue;
import java.util.LinkedList;

// Import the alignment and swerve subsystem packages:
import frc.robot.subsystems.swervedrive.Alignment;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

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

    /**
     * Uses the next target, passes its information to Alignment to compute a SwerveInputStream, then calls
     * driveFieldOriented on the SwerveSubsystem.
     *
     * @param swerveSubsystem The swerve drive subsystem.
     * @param currentPose     The current robot pose.
     */
    public void alignNextTarget(SwerveSubsystem swerveSubsystem, Pose2d currentPose) {
        TargetClass target = getNextTarget();
        if (target != null) {
            // Convert target information into a target pose.
            Pose2d targetPose = target.toPose2d();
            // Calculate the SwerveInputStream using Alignment's driveToPose.
            SwerveInputStream inputStream = Alignment.driveToPose(currentPose, targetPose, swerveSubsystem.getSwerveDrive());
            // Command the drive with the calculated input stream.
            swerveSubsystem.driveFieldOriented(inputStream);
        }
    }

    public boolean hasQueue() {
        return !targetQueue.isEmpty();
    }

    public ButtonBox() {
        // Initialize the ButtonBox subsystem
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}