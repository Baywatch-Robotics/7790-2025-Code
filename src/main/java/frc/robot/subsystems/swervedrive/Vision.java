/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Robot;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    private final PhotonCamera rightCamera;
    private final PhotonCamera leftCamera;
    private final PhotonPoseEstimator leftPhotonEstimator;
    private final PhotonPoseEstimator rightPhotonEstimator;

    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim rightCameraSim;
    private PhotonCameraSim leftCameraSim;
    private VisionSystemSim visionSim;

    private final String kRightCameraName = "rightcam";
    private final String kLeftCameraName = "leftcam";

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final Transform3d rightCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.rightCamXOffset, AprilTagVisionConstants.rightCamYOffset,
                    AprilTagVisionConstants.rightCamZOffset),
            new Rotation3d(AprilTagVisionConstants.rightCamRoll, AprilTagVisionConstants.rightCamPitch,
                    AprilTagVisionConstants.rightCamYaw));

    private static final Transform3d leftCamToRobot = new Transform3d(
            new Translation3d(AprilTagVisionConstants.leftCamXOffset, AprilTagVisionConstants.leftCamYOffset,
                    AprilTagVisionConstants.leftCamZOffset),
            new Rotation3d(AprilTagVisionConstants.leftCamRoll, AprilTagVisionConstants.leftCamPitch,
                    AprilTagVisionConstants.leftCamYaw));

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public Vision() {
        rightCamera = new PhotonCamera(kRightCameraName);
        leftCamera = new PhotonCamera(kLeftCameraName);

        rightPhotonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                rightCamToRobot);

        leftPhotonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                leftCamToRobot);

        leftPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            
            // Create PhotonCameraSim for both cameras
            rightCameraSim = new PhotonCameraSim(rightCamera, cameraProp);
            leftCameraSim = new PhotonCameraSim(leftCamera, cameraProp);
            
            // Add the simulated cameras to view the targets on this simulated field.
            visionSim.addCamera(rightCameraSim, rightCamToRobot);
            visionSim.addCamera(leftCameraSim, leftCamToRobot);

            rightCameraSim.enableDrawWireframe(true);
            leftCameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty.
     * This should only be called once per loop. It tries to get pose estimates from both
     * cameras and returns the best one based on number of tags and their distance.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp,
     *         and targets used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> rightVisionEst = Optional.empty();
        Optional<EstimatedRobotPose> leftVisionEst = Optional.empty();
        
        // Process right camera results
        for (var change : rightCamera.getAllUnreadResults()) {
            rightVisionEst = rightPhotonEstimator.update(change);
            if (rightVisionEst.isPresent()) {
                updateEstimationStdDevs(rightVisionEst, change.getTargets(), rightPhotonEstimator);
            }
        }
        
        // Process left camera results
        for (var change : leftCamera.getAllUnreadResults()) {
            leftVisionEst = leftPhotonEstimator.update(change);
            if (leftVisionEst.isPresent()) {
                updateEstimationStdDevs(leftVisionEst, change.getTargets(), leftPhotonEstimator);
            }
        }
        
        // Determine which estimate to use (prefer the one with more or closer tags)
        Optional<EstimatedRobotPose> bestEstimate = Optional.empty();
        
        // If only one camera has a valid estimate, use it
        if (rightVisionEst.isPresent() && !leftVisionEst.isPresent()) {
            bestEstimate = rightVisionEst;
        } else if (!rightVisionEst.isPresent() && leftVisionEst.isPresent()) {
            bestEstimate = leftVisionEst;
        } else if (rightVisionEst.isPresent() && leftVisionEst.isPresent()) {
            // Both cameras have valid estimates, use the one with more tags or better confidence
            // This is a simple implementation - you might want to implement more sophisticated logic
            if (rightVisionEst.get().targetsUsed.size() > leftVisionEst.get().targetsUsed.size()) {
                bestEstimate = rightVisionEst;
            } else if (rightVisionEst.get().targetsUsed.size() < leftVisionEst.get().targetsUsed.size()) {
                bestEstimate = leftVisionEst;
            } else {
                // Same number of tags, choose based on estimated ambiguity/distance
                // For simplicity, we'll just use right camera in this case
                bestEstimate = rightVisionEst;
            }
        }
        
        if (Robot.isSimulation() && bestEstimate.isPresent()) {
            getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(bestEstimate.get().estimatedPose.toPose2d());
        } else if (Robot.isSimulation()) {
            getSimDebugField().getObject("VisionEstimation").setPoses();
        }
        
        return bestEstimate;
    }

    /**
     * Calculates new standard deviations based on the estimated pose
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     * @param estimator     The estimator that produced this pose
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

}