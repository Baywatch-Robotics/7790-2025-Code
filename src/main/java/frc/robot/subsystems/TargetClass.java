package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class TargetClass {

    // Field width based on blue alliance origin. Update this value as required.
    private static final double FIELD_WIDTH = 17.55;

    private double x;
    private double y;
    private double z;
    private boolean isLeft;
    private int level;
    private int face;
    private boolean isSource;
    private String name;

    public TargetClass(String name) {
        this.name = name;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public boolean isLeft() {
        return isLeft;
    }

    public void setLeft(boolean isLeft) {
        this.isLeft = isLeft;
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public int getFace() {
        return face;
    }

    public void setFace(int face) {
        this.face = face;
    }

    public boolean isSource() {
        return isSource;
    }
    public void setSource(boolean isSource) {
        this.isSource = isSource;
    }
    
    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public static TargetClass GetTargetByName(String name) {
        TargetClass target = new TargetClass(name);
        // Set properties based on the name.
        switch(name) {
            case "SL":
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setLeft(true);
                target.setLevel(0);
                target.setFace(0);
                target.setSource(true);
                break;
            case "SR":
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setLeft(false);
                target.setLevel(0);
                target.setFace(0);
                target.setSource(true);
                break;
            case "C0000":
                target.setX(5.795);
                target.setY(4);
                target.setZ(0);
                target.setLeft(true);
                target.setLevel(0);
                target.setFace(0);
                target.setSource(false);
                break;
            case "C0001":
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setLeft(false);
                target.setLevel(0);
                target.setFace(0);
                target.setSource(false);
                break;
            // Add more cases as needed.
            default:
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setLeft(false);
                target.setLevel(0);
                target.setFace(0);
                target.setSource(false);
                break;
        }
        return target;
    }

    /**
     * Converts this target data into a Pose2d.
     * If on the red alliance, mirror the pose relative to the blue-origin field coordinate system.
     */
    public Pose2d toPose2d() {
        Pose2d pose = new Pose2d(x, y, new Rotation2d());
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            double mirroredX = FIELD_WIDTH - pose.getX();
            double mirroredAngle = Math.PI - pose.getRotation().getRadians();
            return new Pose2d(mirroredX, pose.getY(), new Rotation2d(mirroredAngle));
        }
        return pose;
    }

    @Override
    public String toString() {
        return name;
    }
}