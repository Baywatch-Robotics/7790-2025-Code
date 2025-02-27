package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TargetClassConstants;

public class TargetClass {

    // Field width based on blue alliance origin. Update this value as required.
    private static final double FIELD_WIDTH = 17.55;
    private static final double FIELD_LENGTH = 8.05;

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
                target.setX(1.204f);
                target.setY(7.021f);
                target.setZ(Units.degreesToRadians(-60));
                target.setSource(true);
                break;
            case "SR":
                target.setX(1.204f);
                target.setY(1.045f);
                target.setZ(Units.degreesToRadians(48));
                target.setSource(true);
                break;
            case "C000":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C001":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C010":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C011":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C020":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C021":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C030":    
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C031":
                target.setX(TargetClassConstants.xValueC0XX);
                target.setY(TargetClassConstants.yValueC0XX);
                target.setZ(TargetClassConstants.zValueC0XX);
                target.setFace(TargetClassConstants.faceValueC0XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C100":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C101":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C110":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C111":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C120":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C121":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C130":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C131":
                target.setX(TargetClassConstants.xValueC1XX);
                target.setY(TargetClassConstants.yValueC1XX);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C200":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C201":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C210":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C211":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C220":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C221":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C230":
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C231":        
                target.setX(TargetClassConstants.xValueC2XX);
                target.setY(TargetClassConstants.yValueC2XX);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C300":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C301":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C310":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C311":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C320":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C321":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C330":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C331":
                target.setX(TargetClassConstants.xValueC3XX);
                target.setY(TargetClassConstants.yValueC3XX);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C400":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C401":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C410":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C411":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C420":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C421":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C430":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C431":
                target.setX(TargetClassConstants.xValueC4XX);
                target.setY(TargetClassConstants.yValueC4XX);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C500":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C501":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C510":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C511":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C520":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C521":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C530":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C531":
                target.setX(TargetClassConstants.xValueC5XX);
                target.setY(TargetClassConstants.yValueC5XX);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
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
    public static Pose2d toPose2d(Pose2d currentPose) {

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            double mirroredX = FIELD_WIDTH - currentPose.getX();
            double mirroredY = FIELD_LENGTH - currentPose.getY();
            double mirroredAngle = Math.PI - currentPose.getRotation().getRadians();
            return new Pose2d(mirroredX, mirroredY, new Rotation2d(mirroredAngle));
        }
        return currentPose;
    }

    public static Supplier<Pose2d> toPose2dSupplier(ButtonBox buttonBox) {

        TargetClass currentTarget = buttonBox.peekNextTarget();
        
        Pose2d currentPose = new Pose2d(currentTarget.getX(), currentTarget.getY(), new Rotation2d(currentTarget.getZ()));
        
        return () -> toPose2d(currentPose);
    }
    
    @Override
    public String toString() {
        return name;
    }
}