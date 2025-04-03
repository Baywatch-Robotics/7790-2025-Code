package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.QuestNav;

import static frc.robot.Constants.QuestNavVisionConstants.QUEST_NAV_TO_ROBOT;

public class QuestNavVision extends SubsystemBase {
    private final QuestNav questNav;

    public QuestNavVision() {
        this.questNav = new QuestNav();
    }

    @Override
    public void periodic() {
        // Cleanup QuestNav messages and process heartbeat requests
        questNav.processHeartbeat();
        questNav.cleanUpQuestNavMessages();

        SmartDashboard.putBoolean("Connected", questNav.connected());
        SmartDashboard.putNumber("Battery", questNav.getBatteryPercent());
    }

    public Pair<Pose2d, Double> getPose() {
        // Get the Quest's measured position
        Pose2d questToField = questNav.getPose();
        // Transform the Quest's position to the robot's position
        Pose2d robotToField = questToField.transformBy(QUEST_NAV_TO_ROBOT.inverse());
        // Get the timestamp from the Quest and return the pose with said timestamp
        return new Pair<>(robotToField, questNav.timestamp());
    }

    public void setPose(Pose2d fieldToRobot) {
        // Transform the field relative position to the Quest's relative position
        Pose2d fieldToQuest = fieldToRobot.plus(QUEST_NAV_TO_ROBOT);
        // Reset the Quest's position
        questNav.resetPose(fieldToQuest);
    }
}
