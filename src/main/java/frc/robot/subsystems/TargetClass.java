package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.HashMap;
import java.util.Map;

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

    // Static HashMap for efficient target lookups
    private static final Map<String, TargetConfig> TARGET_CONFIGS = new HashMap<>();
    
    // Helper class to store target configuration
    private static class TargetConfig {
        final double x, y, z;
        final boolean isSource;
        final boolean isLeft;
        final int level, face;
        final boolean useDynamicY;
        
        TargetConfig(double x, double y, double z, boolean isSource, boolean isLeft, int level, int face, boolean useDynamicY) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.isSource = isSource;
            this.isLeft = isLeft;
            this.level = level;
            this.face = face;
            this.useDynamicY = useDynamicY;
        }
        
        TargetConfig(double x, double y, double z, boolean isSource) {
            this(x, y, z, isSource, false, 0, 0, false);
        }
    }
    
    static {
        // Initialize the HashMap with all target configurations
        // Source targets
        TARGET_CONFIGS.put("SL", new TargetConfig(TargetClassConstants.SLPositionX, TargetClassConstants.SLPositionY, TargetClassConstants.SLPositionZ, true));
        TARGET_CONFIGS.put("SR", new TargetConfig(TargetClassConstants.SRPositionX, TargetClassConstants.SRPositionY, TargetClassConstants.SRPositionZ, true));
        TARGET_CONFIGS.put("PL0", new TargetConfig(TargetClassConstants.PLPositionX, TargetClassConstants.PLPositionY, TargetClassConstants.PLPositionZ0, true));
        TARGET_CONFIGS.put("PR0", new TargetConfig(TargetClassConstants.PRPositionX, TargetClassConstants.PRPositionY, TargetClassConstants.PRPositionZ0, true));
        TARGET_CONFIGS.put("PL1", new TargetConfig(TargetClassConstants.PLPositionX, TargetClassConstants.PLPositionY, TargetClassConstants.PLPositionZ1, true));
        TARGET_CONFIGS.put("PR1", new TargetConfig(TargetClassConstants.PRPositionX, TargetClassConstants.PRPositionY, TargetClassConstants.PRPositionZ1, true));
        TARGET_CONFIGS.put("LL", new TargetConfig(TargetClassConstants.LLPositionX, TargetClassConstants.LLPositionY, TargetClassConstants.LLPositionZ, true));
        TARGET_CONFIGS.put("LCR", new TargetConfig(TargetClassConstants.LCRPositionX, TargetClassConstants.LCRPositionY, TargetClassConstants.LCRPositionZ, true));
        TARGET_CONFIGS.put("LCL", new TargetConfig(TargetClassConstants.LCLPositionX, TargetClassConstants.LCLPositionY, TargetClassConstants.LCLPositionZ, true));
        TARGET_CONFIGS.put("LR", new TargetConfig(TargetClassConstants.LRPositionX, TargetClassConstants.LRPositionY, TargetClassConstants.LRPositionZ, true));
        TARGET_CONFIGS.put("Processor", new TargetConfig(TargetClassConstants.ProcessorPositionX, TargetClassConstants.ProcessorPositionY, TargetClassConstants.ProcessorPositionZ, true));
        
        // Dynamic Y targets (A73XX and A83XX series)
        TARGET_CONFIGS.put("A7300", new TargetConfig(TargetClassConstants.xValueA73N0, 0, TargetClassConstants.zValueA73N0, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A7301", new TargetConfig(TargetClassConstants.xValueA73N1, 0, TargetClassConstants.zValueA73N1, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A7310", new TargetConfig(TargetClassConstants.xValueA73N0, 0, TargetClassConstants.zValueA73N0, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A7311", new TargetConfig(TargetClassConstants.xValueA73N1, 0, TargetClassConstants.zValueA73N1, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A8300", new TargetConfig(TargetClassConstants.xValueA83N0, 0, TargetClassConstants.zValueA83N0, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A8301", new TargetConfig(TargetClassConstants.xValueA83N1, 0, TargetClassConstants.zValueA83N1, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A8310", new TargetConfig(TargetClassConstants.xValueA83N0, 0, TargetClassConstants.zValueA83N0, true, false, 0, 0, true));
        TARGET_CONFIGS.put("A8311", new TargetConfig(TargetClassConstants.xValueA83N1, 0, TargetClassConstants.zValueA83N1, true, false, 0, 0, true));
        
        // Coral targets (C series)
        TARGET_CONFIGS.put("C1000", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC10X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1010", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC10X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1100", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1110", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1200", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1210", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1300", new TargetConfig(TargetClassConstants.xValueC130, TargetClassConstants.yValueC130, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1310", new TargetConfig(TargetClassConstants.xValueC131, TargetClassConstants.yValueC131, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C2000", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC20X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2010", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC20X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2100", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X0, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2110", new TargetConfig(TargetClassConstants.xValueC2X1, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2200", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X0, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2210", new TargetConfig(TargetClassConstants.xValueC2X1, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2300", new TargetConfig(TargetClassConstants.xValueC230, TargetClassConstants.yValueC230, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2310", new TargetConfig(TargetClassConstants.xValueC231, TargetClassConstants.yValueC231, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C3000", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC30X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3010", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC30X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3100", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3110", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3200", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3210", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3300", new TargetConfig(TargetClassConstants.xValueC330, TargetClassConstants.yValueC330, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3310", new TargetConfig(TargetClassConstants.xValueC331, TargetClassConstants.yValueC331, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("S3300", new TargetConfig(TargetClassConstants.xValueS330, TargetClassConstants.yValueS330, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("S3310", new TargetConfig(TargetClassConstants.xValueS331, TargetClassConstants.yValueS331, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C4000", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC40X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4010", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC40X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4100", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4110", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4200", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4210", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4300", new TargetConfig(TargetClassConstants.xValueC430, TargetClassConstants.yValueC430, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4310", new TargetConfig(TargetClassConstants.xValueC431, TargetClassConstants.yValueC431, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("S4300", new TargetConfig(TargetClassConstants.xValueS430, TargetClassConstants.yValueS430, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("S4310", new TargetConfig(TargetClassConstants.xValueS431, TargetClassConstants.yValueS431, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C5000", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC50X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5010", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC50X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5100", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5110", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5200", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5210", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5300", new TargetConfig(TargetClassConstants.xValueC530, TargetClassConstants.yValueC530, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5310", new TargetConfig(TargetClassConstants.xValueC531, TargetClassConstants.yValueC531, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("S5300", new TargetConfig(TargetClassConstants.xValueS530, TargetClassConstants.yValueS530, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("S5310", new TargetConfig(TargetClassConstants.xValueS531, TargetClassConstants.yValueS531, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C6000", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC60X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6010", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC60X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6100", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6110", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6200", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6210", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6300", new TargetConfig(TargetClassConstants.xValueC630, TargetClassConstants.yValueC630, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6310", new TargetConfig(TargetClassConstants.xValueC631, TargetClassConstants.yValueC631, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC6XX, false));
        
        // CXXXX1 series (using different z values)
        TARGET_CONFIGS.put("C1001", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC40X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1011", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC40X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1101", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1111", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1201", new TargetConfig(TargetClassConstants.xValueC1X0, TargetClassConstants.yValueC1X0, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1211", new TargetConfig(TargetClassConstants.xValueC1X1, TargetClassConstants.yValueC1X1, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1301", new TargetConfig(TargetClassConstants.xValueC130, TargetClassConstants.yValueC130, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C1311", new TargetConfig(TargetClassConstants.xValueC131, TargetClassConstants.yValueC131, TargetClassConstants.zValueC4XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("C2001", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC50X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2011", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC50X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2101", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X0, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2111", new TargetConfig(TargetClassConstants.xValueC2X1, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2201", new TargetConfig(TargetClassConstants.xValueC2X0, TargetClassConstants.yValueC2X0, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2211", new TargetConfig(TargetClassConstants.xValueC2X1, TargetClassConstants.yValueC2X1, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2301", new TargetConfig(TargetClassConstants.xValueC230, TargetClassConstants.yValueC230, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C2311", new TargetConfig(TargetClassConstants.xValueC231, TargetClassConstants.yValueC231, TargetClassConstants.zValueC5XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("C3001", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC60X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3011", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC60X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3101", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3111", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3201", new TargetConfig(TargetClassConstants.xValueC3X0, TargetClassConstants.yValueC3X0, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3211", new TargetConfig(TargetClassConstants.xValueC3X1, TargetClassConstants.yValueC3X1, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3301", new TargetConfig(TargetClassConstants.xValueC330, TargetClassConstants.yValueC330, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C3311", new TargetConfig(TargetClassConstants.xValueC331, TargetClassConstants.yValueC331, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("S3301", new TargetConfig(TargetClassConstants.xValueS330, TargetClassConstants.yValueS330, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("S3311", new TargetConfig(TargetClassConstants.xValueS331, TargetClassConstants.yValueS331, TargetClassConstants.zValueC6XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("C4001", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC10X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4011", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC10X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4101", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4111", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4201", new TargetConfig(TargetClassConstants.xValueC4X0, TargetClassConstants.yValueC4X0, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4211", new TargetConfig(TargetClassConstants.xValueC4X1, TargetClassConstants.yValueC4X1, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4301", new TargetConfig(TargetClassConstants.xValueC430, TargetClassConstants.yValueC430, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C4311", new TargetConfig(TargetClassConstants.xValueC431, TargetClassConstants.yValueC431, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("S4301", new TargetConfig(TargetClassConstants.xValueS430, TargetClassConstants.yValueS430, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("S4311", new TargetConfig(TargetClassConstants.xValueS431, TargetClassConstants.yValueS431, TargetClassConstants.zValueC1XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("C5001", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC20X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5011", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC20X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5101", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5111", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5201", new TargetConfig(TargetClassConstants.xValueC5X0, TargetClassConstants.yValueC5X0, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5211", new TargetConfig(TargetClassConstants.xValueC5X1, TargetClassConstants.yValueC5X1, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5301", new TargetConfig(TargetClassConstants.xValueC530, TargetClassConstants.yValueC530, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C5311", new TargetConfig(TargetClassConstants.xValueC531, TargetClassConstants.yValueC531, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("S5301", new TargetConfig(TargetClassConstants.xValueS530, TargetClassConstants.yValueS530, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("S5311", new TargetConfig(TargetClassConstants.xValueS531, TargetClassConstants.yValueS531, TargetClassConstants.zValueC2XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("C6001", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC30X, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6011", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC30X, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX0X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6101", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6111", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX1X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6201", new TargetConfig(TargetClassConstants.xValueC6X0, TargetClassConstants.yValueC6X0, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6211", new TargetConfig(TargetClassConstants.xValueC6X1, TargetClassConstants.yValueC6X1, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX2X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6301", new TargetConfig(TargetClassConstants.xValueC630, TargetClassConstants.yValueC630, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX0, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("C6311", new TargetConfig(TargetClassConstants.xValueC631, TargetClassConstants.yValueC631, TargetClassConstants.zValueC3XX, false, TargetClassConstants.setLeftCXX1, TargetClassConstants.heightCX3X, TargetClassConstants.faceValueC6XX, false));
        
        // Algae targets (A series)
        TARGET_CONFIGS.put("A1200", new TargetConfig(TargetClassConstants.xValueA1X0, TargetClassConstants.yValueA1X0, TargetClassConstants.zValueC1XX, false, false, 2, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("A1210", new TargetConfig(TargetClassConstants.xValueA1X1, TargetClassConstants.yValueA1X1, TargetClassConstants.zValueC1XX, false, false, 2, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("A1201", new TargetConfig(TargetClassConstants.xValueA1X0, TargetClassConstants.yValueA1X0, TargetClassConstants.zValueC4XX, false, false, 2, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("A1211", new TargetConfig(TargetClassConstants.xValueA1X1, TargetClassConstants.yValueA1X1, TargetClassConstants.zValueC4XX, false, false, 2, TargetClassConstants.faceValueC1XX, false));
        TARGET_CONFIGS.put("A2100", new TargetConfig(TargetClassConstants.xValueA2X0, TargetClassConstants.yValueA2X0, TargetClassConstants.zValueC2XX, false, false, 1, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("A2110", new TargetConfig(TargetClassConstants.xValueA2X1, TargetClassConstants.yValueA2X1, TargetClassConstants.zValueC2XX, false, false, 1, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("A2101", new TargetConfig(TargetClassConstants.xValueA2X0, TargetClassConstants.yValueA2X0, TargetClassConstants.zValueC5XX, false, false, 1, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("A2111", new TargetConfig(TargetClassConstants.xValueA2X1, TargetClassConstants.yValueA2X1, TargetClassConstants.zValueC5XX, false, false, 1, TargetClassConstants.faceValueC2XX, false));
        TARGET_CONFIGS.put("A3200", new TargetConfig(TargetClassConstants.xValueA3X0, TargetClassConstants.yValueA3X0, TargetClassConstants.zValueC3XX, false, false, 2, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("A3210", new TargetConfig(TargetClassConstants.xValueA3X1, TargetClassConstants.yValueA3X1, TargetClassConstants.zValueC3XX, false, false, 2, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("A3201", new TargetConfig(TargetClassConstants.xValueA3X0, TargetClassConstants.yValueA3X0, TargetClassConstants.zValueC6XX, false, false, 2, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("A3211", new TargetConfig(TargetClassConstants.xValueA3X1, TargetClassConstants.yValueA3X1, TargetClassConstants.zValueC6XX, false, false, 2, TargetClassConstants.faceValueC3XX, false));
        TARGET_CONFIGS.put("A4100", new TargetConfig(TargetClassConstants.xValueA4X0, TargetClassConstants.yValueA4X0, TargetClassConstants.zValueC4XX, false, false, 1, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("A4110", new TargetConfig(TargetClassConstants.xValueA4X1, TargetClassConstants.yValueA4X1, TargetClassConstants.zValueC4XX, false, false, 1, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("A4101", new TargetConfig(TargetClassConstants.xValueA4X0, TargetClassConstants.yValueA4X0, TargetClassConstants.zValueC1XX, false, false, 1, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("A4111", new TargetConfig(TargetClassConstants.xValueA4X1, TargetClassConstants.yValueA4X1, TargetClassConstants.zValueC1XX, false, false, 1, TargetClassConstants.faceValueC4XX, false));
        TARGET_CONFIGS.put("A5200", new TargetConfig(TargetClassConstants.xValueA5X0, TargetClassConstants.yValueA5X0, TargetClassConstants.zValueC5XX, false, false, 2, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("A5210", new TargetConfig(TargetClassConstants.xValueA5X1, TargetClassConstants.yValueA5X1, TargetClassConstants.zValueC5XX, false, false, 2, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("A5201", new TargetConfig(TargetClassConstants.xValueA5X0, TargetClassConstants.yValueA5X0, TargetClassConstants.zValueC2XX, false, false, 2, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("A5211", new TargetConfig(TargetClassConstants.xValueA5X1, TargetClassConstants.yValueA5X1, TargetClassConstants.zValueC2XX, false, false, 2, TargetClassConstants.faceValueC5XX, false));
        TARGET_CONFIGS.put("A6100", new TargetConfig(TargetClassConstants.xValueA6X0, TargetClassConstants.yValueA6X0, TargetClassConstants.zValueC6XX, false, false, 1, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("A6110", new TargetConfig(TargetClassConstants.xValueA6X1, TargetClassConstants.yValueA6X1, TargetClassConstants.zValueC6XX, false, false, 1, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("A6101", new TargetConfig(TargetClassConstants.xValueA6X0, TargetClassConstants.yValueA6X0, TargetClassConstants.zValueC3XX, false, false, 1, TargetClassConstants.faceValueC6XX, false));
        TARGET_CONFIGS.put("A6111", new TargetConfig(TargetClassConstants.xValueA6X1, TargetClassConstants.yValueA6X1, TargetClassConstants.zValueC3XX, false, false, 1, TargetClassConstants.faceValueC6XX, false));
    }

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
        TargetConfig config = TARGET_CONFIGS.get(name);
        
        if (config == null) {
            return null;
        }
        
        TargetClass target = new TargetClass(name);
        target.setX(config.x);
        target.setZ(config.z);
        target.setSource(config.isSource);
        target.setLeft(config.isLeft);
        target.setLevel(config.level);
        target.setFace(config.face);
        
        if (config.useDynamicY) {
            double currentY = robotPoseSupplier.get().getY();
            double flippedY = 2 * CENTERLINE_Y_METERS - currentY;
            target.setY(flippedY);
        } else {
            target.setY(config.y);
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