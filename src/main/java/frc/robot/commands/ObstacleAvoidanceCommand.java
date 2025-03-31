package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ObstacleAvoidanceConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command for navigating to a pose while avoiding obstacles like the reef
 */
public class ObstacleAvoidanceCommand extends Command {
    private final SwerveSubsystem swerve;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final List<Translation2d> waypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;
    private Pose2d finalTargetPose;
    private Command currentCommand = null;
    private boolean isFinished = false;
    private final Field2d field;
    
    /**
     * Creates a command that navigates to a target pose while avoiding obstacles
     * 
     * @param swerve The swerve drive subsystem
     * @param targetPoseSupplier Supplier of the target pose
     */
    public ObstacleAvoidanceCommand(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
        this.swerve = swerve;
        this.targetPoseSupplier = targetPoseSupplier;
        
        field = new Field2d();
        if (ObstacleAvoidanceConstants.DEBUG_MODE) {
            SmartDashboard.putData("Obstacle Avoidance Field", field);
        }
        
        addRequirements(swerve);
    }
    
    /**
     * Creates a command that navigates to a specific target pose while avoiding obstacles
     * 
     * @param swerve The swerve drive subsystem
     * @param targetPose The target pose
     */
    public ObstacleAvoidanceCommand(SwerveSubsystem swerve, Pose2d targetPose) {
        this(swerve, () -> targetPose);
    }
    
    @Override
    public void initialize() {
        // Get the final target pose
        finalTargetPose = targetPoseSupplier.get();
        Pose2d currentPose = swerve.getPose();
        
        // Clear previous waypoints
        waypoints.clear();
        currentWaypointIndex = 0;
        isFinished = false;
        
        SmartDashboard.putString("Obstacle Avoidance Status", "Planning path");
        
        // Plan path around obstacles
        planPath(currentPose, finalTargetPose);
        
        // Start navigation to first waypoint or directly to target if no waypoints needed
        navigateToNextPoint();
        
        // Debug visualization
        if (ObstacleAvoidanceConstants.DEBUG_MODE) {
            updateFieldVisualization();
        }
    }
    
    /**
     * Plans a path from start to target avoiding obstacles
     */
    private void planPath(Pose2d start, Pose2d target) {
        // First check if direct path is clear
        if (isPathClear(start.getTranslation(), target.getTranslation())) {
            // Direct path is clear, no waypoints needed
            SmartDashboard.putString("Path Planning", "Direct path clear");
            return;
        }
        
        // Path is obstructed, we need to find waypoints around the reef
        generateWaypointsAroundReef(start.getTranslation(), target.getTranslation());
    }
    
    /**
     * Checks if a path between two points is clear of obstacles
     */
    private boolean isPathClear(Translation2d start, Translation2d end) {
        // Check if path intersects with the reef
        double pathLength = start.getDistance(end);
        int numChecks = (int) Math.ceil(pathLength / ObstacleAvoidanceConstants.PATH_RESOLUTION);
        
        for (int i = 0; i <= numChecks; i++) {
            double t = (double) i / numChecks;
            Translation2d point = start.interpolate(end, t);
            if (isInReef(point)) {
                SmartDashboard.putNumber("Collision Point X", point.getX());
                SmartDashboard.putNumber("Collision Point Y", point.getY());
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Checks if a point is inside the reef zone
     */
    private boolean isInReef(Translation2d point) {
        return point.getDistance(
            new Translation2d(ObstacleAvoidanceConstants.REEF_CENTER_X, ObstacleAvoidanceConstants.REEF_CENTER_Y)
        ) < ObstacleAvoidanceConstants.REEF_RADIUS;
    }
    
    /**
     * Generates waypoints to navigate around the reef
     */
    private void generateWaypointsAroundReef(Translation2d start, Translation2d end) {
        Translation2d reefCenter = new Translation2d(
            ObstacleAvoidanceConstants.REEF_CENTER_X, 
            ObstacleAvoidanceConstants.REEF_CENTER_Y
        );
        
        double reefRadius = ObstacleAvoidanceConstants.REEF_RADIUS;
        
        // Calculate tangent points from both start and end positions to reef
        List<Translation2d> startTangentPoints = calculateTangentPoints(start, reefCenter, reefRadius);
        List<Translation2d> endTangentPoints = calculateTangentPoints(end, reefCenter, reefRadius);
        
        if (startTangentPoints.isEmpty() || endTangentPoints.isEmpty()) {
            // Cannot find valid tangent points, use longer path around reef
            generateSimpleAvoidancePath(start, end, reefCenter, reefRadius);
            return;
        }
        
        // Find best pair of tangent points (one from each list)
        Translation2d bestStartTangent = null;
        Translation2d bestEndTangent = null;
        double bestPathLength = Double.MAX_VALUE;
        
        for (Translation2d startTangent : startTangentPoints) {
            for (Translation2d endTangent : endTangentPoints) {
                // Check if the arc between tangent points and the straight lines are valid
                if (isValidTangentPath(startTangent, endTangent, reefCenter, reefRadius)) {
                    // Calculate total path length: start->startTangent + arc + endTangent->end
                    double arcLength = calculateArcLength(startTangent, endTangent, reefCenter, reefRadius);
                    double totalLength = start.getDistance(startTangent) + arcLength + endTangent.getDistance(end);
                    
                    if (totalLength < bestPathLength) {
                        bestStartTangent = startTangent;
                        bestEndTangent = endTangent;
                        bestPathLength = totalLength;
                    }
                }
            }
        }
        
        // If valid tangent path found
        if (bestStartTangent != null && bestEndTangent != null) {
            // Add waypoints along the path
            waypoints.add(bestStartTangent);
            
            // Add intermediate waypoints along the arc if necessary
            List<Translation2d> arcWaypoints = generateArcWaypoints(
                bestStartTangent, bestEndTangent, reefCenter, reefRadius);
            waypoints.addAll(arcWaypoints);
            
            waypoints.add(bestEndTangent);
        } else {
            // Fallback to simple avoidance
            generateSimpleAvoidancePath(start, end, reefCenter, reefRadius);
        }
    }
    
    /**
     * Calculate tangent points from a point to a circle
     */
    private List<Translation2d> calculateTangentPoints(Translation2d point, Translation2d center, double radius) {
        List<Translation2d> tangentPoints = new ArrayList<>();
        
        double dx = point.getX() - center.getX();
        double dy = point.getY() - center.getY();
        double distanceSquared = dx * dx + dy * dy;
        double distance = Math.sqrt(distanceSquared);
        
        // If point is inside the circle (accounting for buffer), no tangent exists
        if (distance < radius) {
            return tangentPoints;
        }
        
        // Calculate tangent points
        double radiusWithBuffer = radius + ObstacleAvoidanceConstants.TANGENT_POINT_BUFFER;
        double a = (radius * radius) / distanceSquared;
        double b = (radius * Math.sqrt(distanceSquared - radius * radius)) / distanceSquared;
        
        // Calculate tangent points
        double x1 = center.getX() + a * dx + b * dy;
        double y1 = center.getY() + a * dy - b * dx;
        tangentPoints.add(new Translation2d(x1, y1));
        
        double x2 = center.getX() + a * dx - b * dy;
        double y2 = center.getY() + a * dy + b * dx;
        tangentPoints.add(new Translation2d(x2, y2));
        
        return tangentPoints;
    }
    
    /**
     * Check if the path between two tangent points is valid (doesn't cross through the circle)
     */
    private boolean isValidTangentPath(Translation2d tangent1, Translation2d tangent2, 
                                      Translation2d center, double radius) {
        // Check if direct path between tangents crosses through circle
        // For tangent points, we need to check if they follow a valid arc around the circle
        
        // Calculate vectors from center to tangent points
        Translation2d v1 = tangent1.minus(center);
        Translation2d v2 = tangent2.minus(center);
        
        // Calculate cross product to determine orientation
        double crossProduct = v1.getX() * v2.getY() - v1.getY() * v2.getX();
        
        // The tangent path is valid if both points are on same side of circle
        // This is a simplified check - in a real implementation you would need more thorough validation
        return crossProduct >= 0;
    }
    
    /**
     * Calculate the arc length between two points on the circle
     */
    private double calculateArcLength(Translation2d p1, Translation2d p2, Translation2d center, double radius) {
        // Calculate vectors from center to points
        Translation2d v1 = p1.minus(center);
        Translation2d v2 = p2.minus(center);
        
        // Calculate the angle between vectors
        double dot = v1.getX() * v2.getX() + v1.getY() * v2.getY();
        double angle = Math.acos(dot / (v1.getNorm() * v2.getNorm()));
        
        // Arc length = radius * angle
        return radius * angle;
    }
    
    /**
     * Generate waypoints along an arc between two points
     */
    private List<Translation2d> generateArcWaypoints(Translation2d start, Translation2d end, 
                                                   Translation2d center, double radius) {
        List<Translation2d> arcPoints = new ArrayList<>();
        
        // Calculate vectors from center to points
        Translation2d v1 = start.minus(center);
        Translation2d v2 = end.minus(center);
        
        // Calculate angle between vectors
        double dot = v1.getX() * v2.getX() + v1.getY() * v2.getY();
        double angleBetween = Math.acos(dot / (v1.getNorm() * v2.getNorm()));
        
        // Determine direction (clockwise/counterclockwise)
        double cross = v1.getX() * v2.getY() - v1.getY() * v2.getX();
        int direction = cross >= 0 ? 1 : -1;
        
        // Calculate start angle
        double startAngle = Math.atan2(v1.getY(), v1.getX());
        
        // Number of waypoints based on angle
        int numWaypoints = (int)Math.max(1, Math.ceil(angleBetween / Math.PI * 4));
        numWaypoints = Math.min(numWaypoints, 3); // Limit to 3 waypoints max
        
        // Generate intermediate waypoints
        for (int i = 1; i < numWaypoints; i++) {
            double t = (double) i / numWaypoints;
            double angle = startAngle + direction * angleBetween * t;
            
            double x = center.getX() + radius * Math.cos(angle);
            double y = center.getY() + radius * Math.sin(angle);
            
            arcPoints.add(new Translation2d(x, y));
        }
        
        return arcPoints;
    }
    
    /**
     * Generate a simple path to avoid the reef
     * This is used as a fallback when tangent-based approach doesn't work
     */
    private void generateSimpleAvoidancePath(Translation2d start, Translation2d end, 
                                           Translation2d reefCenter, double reefRadius) {
        // Calculate line from start to end
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double length = Math.sqrt(dx * dx + dy * dy);
        
        // Normalize the direction vector
        dx /= length;
        dy /= length;
        
        // Get perpendicular vector (rotate 90 degrees)
        double perpX = -dy;
        double perpY = dx;
        
        // Calculate vector from reef center to the line
        double centerToLineX = reefCenter.getX() - start.getX();
        double centerToLineY = reefCenter.getY() - start.getY();
        
        // Project this vector onto the perpendicular to find distance to line
        double projection = centerToLineX * perpX + centerToLineY * perpY;
        
        // Determine which side of the line the reef is on
        int side = projection >= 0 ? 1 : -1;
        
        // Go in the opposite direction from the reef
        double avoidanceDistance = reefRadius + ObstacleAvoidanceConstants.OBSTACLE_CLEARANCE;
        
        // Create midpoint for avoidance
        double midX = (start.getX() + end.getX()) / 2;
        double midY = (start.getY() + end.getY()) / 2;
        
        // Add offset away from reef
        midX += -side * perpX * avoidanceDistance;
        midY += -side * perpY * avoidanceDistance;
        
        // Add midpoint as waypoint
        waypoints.add(new Translation2d(midX, midY));
    }
    
    @Override
    public void execute() {
        // If currently executing a ProfileToPose command, let it run
        if (currentCommand != null && !currentCommand.isFinished()) {
            return;
        }
        
        // Previous command finished, move to next waypoint
        if (currentCommand != null && currentCommand.isFinished()) {
            currentWaypointIndex++;
            navigateToNextPoint();
        }
        
        // Update visualization
        if (ObstacleAvoidanceConstants.DEBUG_MODE) {
            updateFieldVisualization();
        }
    }
    
    /**
     * Navigate to the next waypoint or final destination
     */
    private void navigateToNextPoint() {
        if (currentWaypointIndex >= waypoints.size()) {
            // All waypoints have been visited, navigate to final destination
            Pose2d targetPose = finalTargetPose;
            SmartDashboard.putString("Obstacle Avoidance Status", "Navigating to final destination");
            SmartDashboard.putString("Current Target", "Final: " + targetPose.getX() + ", " + targetPose.getY());
            
            // Create a new ProfileToPose command to the final destination with isFinalDestination=true
            currentCommand = new ProfileToPose(swerve, () -> targetPose, true);
            CommandScheduler.getInstance().schedule(currentCommand);
            
            // Mark as on final leg
            if (waypoints.isEmpty()) {
                // If there were no waypoints, this is a direct path
                isFinished = true;  // Command will complete when the ProfileToPose finishes
            }
        } else {
            // Navigate to the next waypoint
            Translation2d nextWaypoint = waypoints.get(currentWaypointIndex);
            
            // Create a pose at the waypoint with rotation pointing toward final target
            Translation2d finalPosition = finalTargetPose.getTranslation();
            Translation2d directionVector = finalPosition.minus(nextWaypoint);
            Rotation2d waypointRotation = new Rotation2d(Math.atan2(directionVector.getY(), directionVector.getX()));
            
            // Create a pose at the waypoint
            Pose2d waypointPose = new Pose2d(nextWaypoint, waypointRotation);
            
            SmartDashboard.putString("Obstacle Avoidance Status", 
                "Navigating to waypoint " + (currentWaypointIndex + 1) + " of " + waypoints.size());
            SmartDashboard.putString("Current Target", 
                "Waypoint " + (currentWaypointIndex + 1) + ": " + nextWaypoint.getX() + ", " + nextWaypoint.getY());
            
            // Create a new ProfileToPose command to this waypoint with isFinalDestination=false
            currentCommand = new ProfileToPose(swerve, () -> waypointPose, false);
            CommandScheduler.getInstance().schedule(currentCommand);
        }
    }
    
    /**
     * Update field visualization to show the planned path
     */
    private void updateFieldVisualization() {
        // Get current pose for reference
        Pose2d currentPose = swerve.getPose();
        
        // Create an array to hold all poses to display
        List<Pose2d> pathPoses = new ArrayList<>();
        pathPoses.add(currentPose); // Start with current position
        
        // Add all waypoints with the rotation facing the next waypoint
        for (int i = 0; i < waypoints.size(); i++) {
            Translation2d waypointPosition = waypoints.get(i);
            
            // Figure out rotation direction - either toward next waypoint or final target
            Translation2d nextPosition;
            if (i < waypoints.size() - 1) {
                nextPosition = waypoints.get(i + 1);
            } else {
                nextPosition = finalTargetPose.getTranslation();
            }
            
            // Calculate direction to next point
            Translation2d directionVector = nextPosition.minus(waypointPosition);
            Rotation2d waypointRotation = new Rotation2d(Math.atan2(directionVector.getY(), directionVector.getX()));
            
            // Add the pose to our list
            pathPoses.add(new Pose2d(waypointPosition, waypointRotation));
        }
        
        // Add final destination
        pathPoses.add(finalTargetPose);
        
        // Display current waypoint with a different color/marking
        if (currentWaypointIndex < waypoints.size()) {
            Translation2d currentWaypointPos = waypoints.get(currentWaypointIndex);
            Pose2d currentWaypointPose = new Pose2d(
                currentWaypointPos,
                new Rotation2d(0)  // Arbitrary rotation for display
            );
            field.getObject("Current Waypoint").setPose(currentWaypointPose);
        } else if (!waypoints.isEmpty()) {
            // We're navigating to final goal, highlight it
            field.getObject("Current Waypoint").setPose(finalTargetPose);
        }
        
        // Display all waypoints and path
        field.getObject("Path Poses").setPoses(pathPoses);
        
        // Draw reef obstacle
        Translation2d reefCenter = new Translation2d(
            ObstacleAvoidanceConstants.REEF_CENTER_X,
            ObstacleAvoidanceConstants.REEF_CENTER_Y
        );
        Pose2d reefPose = new Pose2d(reefCenter, new Rotation2d(0));
        field.getObject("Reef").setPose(reefPose);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Clean up any running commands
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
        
        // Clear visualization
        if (ObstacleAvoidanceConstants.DEBUG_MODE) {
            field.getObject("Path Poses").setPoses();
            field.getObject("Current Waypoint").setPose(new Pose2d());
        }
        
        SmartDashboard.putString("Obstacle Avoidance Status", interrupted ? "Interrupted" : "Completed");
    }
    
    @Override
    public boolean isFinished() {
        // If we're on the last navigation command (to the final destination)
        // and it has finished, then we're done
        if (currentWaypointIndex >= waypoints.size() && currentCommand != null) {
            return currentCommand.isFinished();
        }
        
        // Special case for direct path with no waypoints
        if (isFinished && currentCommand != null) {
            return currentCommand.isFinished();
        }
        
        // Otherwise, we're not finished until we've navigated through all waypoints
        // and completed the final navigation command
        return false;
    }
}
