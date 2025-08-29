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
    // Centerline (158.5 inches) converted to meters for Y reflection of AX3X0/AX3X1
    private static final double CENTERLINE_Y_METERS = Units.inchesToMeters(158.5);

    private double x;
    private double y;
    private double z;
    private boolean isLeft;
    private int level;
    private int face;
    private boolean isSource;
    private String name;

    // Supplier for current robot pose (set by SwerveSubsystem)
    private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

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

    /**
     * Register a supplier that returns the current robot pose.
     * Must be called during robot init (done in SwerveSubsystem).
     */
    public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        if (supplier != null) robotPoseSupplier = supplier;
    }

    public static TargetClass GetTargetByName(String name) {
        TargetClass target = new TargetClass(name);
        // Set properties based on the name.

        //Face 0-5
        //Level 0-3
        //0 left, 1 right

        switch(name) {
            case "SL":
                target.setX(TargetClassConstants.SLPositionX);
                target.setY(TargetClassConstants.SLPositionY);
                target.setZ(TargetClassConstants.SLPositionZ);
                target.setSource(true);
                break;
            case "SR":
                target.setX(TargetClassConstants.SRPositionX);
                target.setY(TargetClassConstants.SRPositionY);
                target.setZ(TargetClassConstants.SRPositionZ);
                target.setSource(true);
                break;
            case "CL":
                target.setX(TargetClassConstants.CLPositionX);
                target.setY(TargetClassConstants.CLPositionY);
                target.setZ(TargetClassConstants.CLPositionZ);
                target.setSource(true);
                break;
            case "CC":
                target.setX(TargetClassConstants.CCPositionX);
                target.setY(TargetClassConstants.CCPositionY);
                target.setZ(TargetClassConstants.CCPositionZ);
                target.setSource(true);
                break;
            case "CR":
                target.setX(TargetClassConstants.CRPositionX);
                target.setY(TargetClassConstants.CRPositionY);
                target.setZ(TargetClassConstants.CRPositionZ);
                target.setSource(true);
                break;
            case "Processor":
                target.setX(TargetClassConstants.ProcessorPositionX);
                target.setY(TargetClassConstants.ProcessorPositionY);
                target.setZ(TargetClassConstants.ProcessorPositionZ);
                target.setSource(true);
                break;
            case "A7300":
                target.setX(TargetClassConstants.xValueA73N0);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA73N0);
                target.setSource(true);
                break;
            case "A7301":
                target.setX(TargetClassConstants.xValueA73N1);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA73N1);
                target.setSource(true);
                break;
            case "A7310":
                target.setX(TargetClassConstants.xValueA73N0);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA73N0);
                target.setSource(true);
                break;
            case "A7311":
                target.setX(TargetClassConstants.xValueA73N1);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA73N1);
                target.setSource(true);
                break;
            case "A8300":
                target.setX(TargetClassConstants.xValueA83N0);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA83N0);
                target.setSource(true);
                break;
            case "A8301":
                target.setX(TargetClassConstants.xValueA83N1);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA83N1);
                target.setSource(true);
                break;
            case "A8310":
                target.setX(TargetClassConstants.xValueA83N0);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA83N0);
                target.setSource(true);
                break;
            case "A8311":
                target.setX(TargetClassConstants.xValueA83N1);
                {
                    double currentY = robotPoseSupplier.get().getY();
                    double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
                    target.setY(flippedY);
                }
                target.setZ(TargetClassConstants.zValueA83N1);
                target.setSource(true);
                break;
            case "C1000":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC10X);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C1010":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC10X);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C1100":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C1110":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C1200":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C1210":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C1300":
                target.setX(TargetClassConstants.xValueC130);
                target.setY(TargetClassConstants.yValueC130);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C1310":
                target.setX(TargetClassConstants.xValueC131);
                target.setY(TargetClassConstants.yValueC131);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C2000":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC20X);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C2010":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC20X);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C2100":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C2110":
                target.setX(TargetClassConstants.xValueC2X1);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C2200":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C2210":
                target.setX(TargetClassConstants.xValueC2X1);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C2300":
                target.setX(TargetClassConstants.xValueC230);
                target.setY(TargetClassConstants.yValueC230);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C2310":        
                target.setX(TargetClassConstants.xValueC231);
                target.setY(TargetClassConstants.yValueC231);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C3000":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC30X);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C3010":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC30X);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C3100":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C3110":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C3200":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C3210":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C3300":
                target.setX(TargetClassConstants.xValueC330);
                target.setY(TargetClassConstants.yValueC330);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C3310":
                target.setX(TargetClassConstants.xValueC331);
                target.setY(TargetClassConstants.yValueC331);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S3300":
                target.setX(TargetClassConstants.xValueS330);
                target.setY(TargetClassConstants.yValueS330);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S3310":
                target.setX(TargetClassConstants.xValueS331);
                target.setY(TargetClassConstants.yValueS331);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C4000":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC40X);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C4010":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC40X);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C4100":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C4110":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C4200":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C4210":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C4300":
                target.setX(TargetClassConstants.xValueC430);
                target.setY(TargetClassConstants.yValueC430);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C4310":
                target.setX(TargetClassConstants.xValueC431);
                target.setY(TargetClassConstants.yValueC431);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S4300":
                target.setX(TargetClassConstants.xValueS430);
                target.setY(TargetClassConstants.yValueS430);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S4310":
                target.setX(TargetClassConstants.xValueS431);
                target.setY(TargetClassConstants.yValueS431);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C5000":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC50X);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C5010":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC50X);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C5100":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C5110":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C5200":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C5210":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C5300":
                target.setX(TargetClassConstants.xValueC530);
                target.setY(TargetClassConstants.yValueC530);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C5310":
                target.setX(TargetClassConstants.xValueC531);
                target.setY(TargetClassConstants.yValueC531);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S5300":
                target.setX(TargetClassConstants.xValueS530);
                target.setY(TargetClassConstants.yValueS530);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S5310":
                target.setX(TargetClassConstants.xValueS531);
                target.setY(TargetClassConstants.yValueS531);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C6000":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC60X);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C6010":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC60X);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C6100":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C6110":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C6200":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C6210":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C6300":
                target.setX(TargetClassConstants.xValueC630);
                target.setY(TargetClassConstants.yValueC630);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C6310":
                target.setX(TargetClassConstants.xValueC631);
                target.setY(TargetClassConstants.yValueC631);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;



                
            case "C1001":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC40X);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C1011":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC40X);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C1101":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C1111":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C1201":
                target.setX(TargetClassConstants.xValueC1X0);
                target.setY(TargetClassConstants.yValueC1X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C1211":
                target.setX(TargetClassConstants.xValueC1X1);
                target.setY(TargetClassConstants.yValueC1X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C1301":
                target.setX(TargetClassConstants.xValueC130);
                target.setY(TargetClassConstants.yValueC130);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C1311":
                target.setX(TargetClassConstants.xValueC131);
                target.setY(TargetClassConstants.yValueC131);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C2001":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC50X);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C2011":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC50X);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C2101":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C2111":
                target.setX(TargetClassConstants.xValueC2X1);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C2201":
                target.setX(TargetClassConstants.xValueC2X0);
                target.setY(TargetClassConstants.yValueC2X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C2211":
                target.setX(TargetClassConstants.xValueC2X1);
                target.setY(TargetClassConstants.yValueC2X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C2301":
                target.setX(TargetClassConstants.xValueC230);
                target.setY(TargetClassConstants.yValueC230);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C2311":        
                target.setX(TargetClassConstants.xValueC231);
                target.setY(TargetClassConstants.yValueC231);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C3001":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC60X);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C3011":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC60X);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C3101":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C3111":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C3201":
                target.setX(TargetClassConstants.xValueC3X0);
                target.setY(TargetClassConstants.yValueC3X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C3211":
                target.setX(TargetClassConstants.xValueC3X1);
                target.setY(TargetClassConstants.yValueC3X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C3301":
                target.setX(TargetClassConstants.xValueC330);
                target.setY(TargetClassConstants.yValueC330);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C3311":
                target.setX(TargetClassConstants.xValueC331);
                target.setY(TargetClassConstants.yValueC331);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S3301":
                target.setX(TargetClassConstants.xValueS330);
                target.setY(TargetClassConstants.yValueS330);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S3311":
                target.setX(TargetClassConstants.xValueS331);
                target.setY(TargetClassConstants.yValueS331);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C4001":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC10X);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C4011":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC10X);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C4101":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C4111":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C4201":
                target.setX(TargetClassConstants.xValueC4X0);
                target.setY(TargetClassConstants.yValueC4X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C4211":
                target.setX(TargetClassConstants.xValueC4X1);
                target.setY(TargetClassConstants.yValueC4X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C4301":
                target.setX(TargetClassConstants.xValueC430);
                target.setY(TargetClassConstants.yValueC430);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C4311":
                target.setX(TargetClassConstants.xValueC431);
                target.setY(TargetClassConstants.yValueC431);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S4301":
                target.setX(TargetClassConstants.xValueS430);
                target.setY(TargetClassConstants.yValueS430);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S4311":
                target.setX(TargetClassConstants.xValueS431);
                target.setY(TargetClassConstants.yValueS431);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C5001":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC20X);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C5011":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC20X);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C5101":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C5111":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C5201":
                target.setX(TargetClassConstants.xValueC5X0);
                target.setY(TargetClassConstants.yValueC5X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C5211":
                target.setX(TargetClassConstants.xValueC5X1);
                target.setY(TargetClassConstants.yValueC5X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C5301":
                target.setX(TargetClassConstants.xValueC530);
                target.setY(TargetClassConstants.yValueC530);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C5311":
                target.setX(TargetClassConstants.xValueC531);
                target.setY(TargetClassConstants.yValueC531);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S5301":
                target.setX(TargetClassConstants.xValueS530);
                target.setY(TargetClassConstants.yValueS530);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "S5311":
                target.setX(TargetClassConstants.xValueS531);
                target.setY(TargetClassConstants.yValueS531);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C6001":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC30X);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C6011":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC30X);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX0X);
                target.setSource(false);
                break;
            case "C6101":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C6111":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX1X);
                target.setSource(false);
                break;
            case "C6201":
                target.setX(TargetClassConstants.xValueC6X0);
                target.setY(TargetClassConstants.yValueC6X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C6211":
                target.setX(TargetClassConstants.xValueC6X1);
                target.setY(TargetClassConstants.yValueC6X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX2X);
                target.setSource(false);
                break;
            case "C6301":
                target.setX(TargetClassConstants.xValueC630);
                target.setY(TargetClassConstants.yValueC630);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX0);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;
            case "C6311":
                target.setX(TargetClassConstants.xValueC631);
                target.setY(TargetClassConstants.yValueC631);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLeft(TargetClassConstants.setLeftCXX1);
                target.setLevel(TargetClassConstants.heightCX3X);
                target.setSource(false);
                break;


                
            // Algae targets for Face 1
            case "A1200": // Face 1 with backup
                target.setX(TargetClassConstants.xValueA1X0);
                target.setY(TargetClassConstants.yValueA1X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A1210": // Face 1 no backup
                target.setX(TargetClassConstants.xValueA1X1);
                target.setY(TargetClassConstants.yValueA1X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A1201": // Face 1 with backup
                target.setX(TargetClassConstants.xValueA1X0);
                target.setY(TargetClassConstants.yValueA1X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A1211": // Face 1 no backup
                target.setX(TargetClassConstants.xValueA1X1);
                target.setY(TargetClassConstants.yValueA1X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC1XX);
                target.setLevel(2);
                target.setSource(false);
                break;
                
            // Algae targets for Face 2
            case "A2100": // Face 2 with backup
                target.setX(TargetClassConstants.xValueA2X0);
                target.setY(TargetClassConstants.yValueA2X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A2110": // Face 2 no backup
                target.setX(TargetClassConstants.xValueA2X1);
                target.setY(TargetClassConstants.yValueA2X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A2101": // Face 2 with backup
                target.setX(TargetClassConstants.xValueA2X0);
                target.setY(TargetClassConstants.yValueA2X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A2111": // Face 2 no backup
                target.setX(TargetClassConstants.xValueA2X1);
                target.setY(TargetClassConstants.yValueA2X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC2XX);
                target.setLevel(1);
                target.setSource(false);
                break;
                
            // Algae targets for Face 3
            case "A3200": // Face 3 with backup
                target.setX(TargetClassConstants.xValueA3X0);
                target.setY(TargetClassConstants.yValueA3X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A3210": // Face 3 no backup
                target.setX(TargetClassConstants.xValueA3X1);
                target.setY(TargetClassConstants.yValueA3X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A3201": // Face 3 with backup
                target.setX(TargetClassConstants.xValueA3X0);
                target.setY(TargetClassConstants.yValueA3X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A3211": // Face 3 no backup
                target.setX(TargetClassConstants.xValueA3X1);
                target.setY(TargetClassConstants.yValueA3X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC3XX);
                target.setLevel(2);
                target.setSource(false);
                break;
                
            // Algae targets for Face 4
            case "A4100": // Face 4 with backup
                target.setX(TargetClassConstants.xValueA4X0);
                target.setY(TargetClassConstants.yValueA4X0);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A4110": // Face 4 no backup
                target.setX(TargetClassConstants.xValueA4X1);
                target.setY(TargetClassConstants.yValueA4X1);
                target.setZ(TargetClassConstants.zValueC4XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A4101": // Face 4 with backup
                target.setX(TargetClassConstants.xValueA4X0);
                target.setY(TargetClassConstants.yValueA4X0);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A4111": // Face 4 no backup
                target.setX(TargetClassConstants.xValueA4X1);
                target.setY(TargetClassConstants.yValueA4X1);
                target.setZ(TargetClassConstants.zValueC1XX);
                target.setFace(TargetClassConstants.faceValueC4XX);
                target.setLevel(1);
                target.setSource(false);
                break;
                
            // Algae targets for Face 5
            case "A5200": // Face 5 with backup
                target.setX(TargetClassConstants.xValueA5X0);
                target.setY(TargetClassConstants.yValueA5X0);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A5210": // Face 5 no backup
                target.setX(TargetClassConstants.xValueA5X1);
                target.setY(TargetClassConstants.yValueA5X1);
                target.setZ(TargetClassConstants.zValueC5XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A5201": // Face 5 with backup
                target.setX(TargetClassConstants.xValueA5X0);
                target.setY(TargetClassConstants.yValueA5X0);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLevel(2);
                target.setSource(false);
                break;
            case "A5211": // Face 5 no backup
                target.setX(TargetClassConstants.xValueA5X1);
                target.setY(TargetClassConstants.yValueA5X1);
                target.setZ(TargetClassConstants.zValueC2XX);
                target.setFace(TargetClassConstants.faceValueC5XX);
                target.setLevel(2);
                target.setSource(false);
                break;
                
            // Algae targets for Face 6
            case "A6100": // Face 6 with backup
                target.setX(TargetClassConstants.xValueA6X0);
                target.setY(TargetClassConstants.yValueA6X0);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A6110": // Face 6 no backup
                target.setX(TargetClassConstants.xValueA6X1);
                target.setY(TargetClassConstants.yValueA6X1);
                target.setZ(TargetClassConstants.zValueC6XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A6101": // Face 6 with backup
                target.setX(TargetClassConstants.xValueA6X0);
                target.setY(TargetClassConstants.yValueA6X0);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            case "A6111": // Face 6 no backup
                target.setX(TargetClassConstants.xValueA6X1);
                target.setY(TargetClassConstants.yValueA6X1);
                target.setZ(TargetClassConstants.zValueC3XX);
                target.setFace(TargetClassConstants.faceValueC6XX);
                target.setLevel(1);
                target.setSource(false);
                break;
            // Add more cases as needed.
            default:
                target = null;
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
            double mirroredAngle = currentPose.getRotation().getRadians() - Math.PI;
            return new Pose2d(mirroredX, mirroredY, new Rotation2d(mirroredAngle));
        }
        return currentPose;
    }

    public static Supplier<Pose2d> toPose2dSupplier(ButtonBox buttonBox) {
        TargetClass currentTarget = buttonBox.peekNextTarget();
        
        
        if (currentTarget == null) {
            
            return () -> new Pose2d();
        }
        
        Pose2d currentPose = new Pose2d(currentTarget.getX(), currentTarget.getY(), new Rotation2d(currentTarget.getZ()));
        return () -> toPose2d(currentPose);
    }
    
    @Override
    public String toString() {
        return name;
    }
}