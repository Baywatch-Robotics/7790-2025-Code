package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ButtonBoxConstants;

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
    public int getFace() {
        return face;
    }
    public void setFace(int face) {
        this.face = face;
    }
    public static TargetClass GetTargetByName(String name) {
        TargetClass target = new TargetClass(name);
        // Set properties based on the name.

        //Face 0-5
        //Level 0-3
        //0 left, 1 right

        switch(name) {
            case "SL":
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setSource(true);
                break;
            case "SR":
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setSource(true);
                break;
            case "C000":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C001":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C010":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C011":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C020":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C021":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C030":    
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C031":
                target.setX(ButtonBoxConstants.xValueC0XX);
                target.setY(ButtonBoxConstants.yValueC0XX);
                target.setZ(ButtonBoxConstants.zValueC0XX);
                target.setFace(ButtonBoxConstants.faceValueC0XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C100":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C101":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C110":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C111":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C120":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C121":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C130":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C131":
                target.setX(ButtonBoxConstants.xValueC1XX);
                target.setY(ButtonBoxConstants.yValueC1XX);
                target.setZ(ButtonBoxConstants.zValueC1XX);
                target.setFace(ButtonBoxConstants.faceValueC1XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C200":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C201":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C210":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C211":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C220":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C221":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C230":
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C231":        
                target.setX(ButtonBoxConstants.xValueC2XX);
                target.setY(ButtonBoxConstants.yValueC2XX);
                target.setZ(ButtonBoxConstants.zValueC2XX);
                target.setFace(ButtonBoxConstants.faceValueC2XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C300":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C301":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C310":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C311":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C320":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C321":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C330":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C331":
                target.setX(ButtonBoxConstants.xValueC3XX);
                target.setY(ButtonBoxConstants.yValueC3XX);
                target.setZ(ButtonBoxConstants.zValueC3XX);
                target.setFace(ButtonBoxConstants.faceValueC3XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C400":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C401":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C410":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C411":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C420":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C421":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C430":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C431":
                target.setX(ButtonBoxConstants.xValueC4XX);
                target.setY(ButtonBoxConstants.yValueC4XX);
                target.setZ(ButtonBoxConstants.zValueC4XX);
                target.setFace(ButtonBoxConstants.faceValueC4XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C500":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C501":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C510":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C511":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C520":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C521":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C530":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX0);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C531":
                target.setX(ButtonBoxConstants.xValueC5XX);
                target.setY(ButtonBoxConstants.yValueC5XX);
                target.setZ(ButtonBoxConstants.zValueC5XX);
                target.setFace(ButtonBoxConstants.faceValueC5XX);
                target.setLeft(ButtonBoxConstants.setLeftCXX1);
                target.setLevel(ButtonBoxConstants.heightCX3X);
                target.setSource(false);
                break;
            // Add more cases as needed.
            default:
                target.setX(0);
                target.setY(0);
                target.setZ(0);
                target.setLeft(false);
                target.setLevel(0);
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