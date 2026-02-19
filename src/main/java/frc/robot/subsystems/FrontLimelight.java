package frc.robot.subsystems;

import static frc.robot.utils.Constants.ObstacleDetectionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunnelPath.Corridor;

/**
 * Front-mounted Limelight camera subsystem for robot obstacle detection.
 * Reads detection data from Limelight's NetworkTables JSON output.
 */
public class FrontLimelight extends SubsystemBase {

    // NetworkTables
    private final NetworkTable m_limelightTable;
    private final NetworkTableEntry m_jsonEntry;
    private final NetworkTableEntry m_tvEntry;  // Target valid

    // Cached detection results
    private List<DetectedRobot> m_detectedRobots = new ArrayList<>();

    // Current robot pose (updated via updateDetectedRobots)
    private Pose2d m_robotPose = new Pose2d();

    // Simulation mock obstacles (for testing without real camera)
    private List<DetectedRobot> m_simMockRobots = new ArrayList<>();
    private boolean m_useSimMock = false;

    // Camera mounting parameters
    private final double m_cameraHeight;      // Height from ground (meters)
    private final double m_cameraPitch;       // Pitch angle (radians, negative = down)
    private final double m_cameraForwardOffset; // Forward offset from robot center (meters)

    // Regex patterns for JSON parsing
    private static final Pattern DETECTOR_PATTERN = Pattern.compile(
        "\"class\"\\s*:\\s*\"([^\"]+)\".*?\"conf\"\\s*:\\s*([\\d.]+).*?\"tx\"\\s*:\\s*([\\d.-]+).*?\"ty\"\\s*:\\s*([\\d.-]+)",
        Pattern.DOTALL
    );

    public FrontLimelight() {
        // Connect to Limelight NetworkTables
        m_limelightTable = NetworkTableInstance.getDefault().getTable(k_frontCameraName);
        m_jsonEntry = m_limelightTable.getEntry("json");
        m_tvEntry = m_limelightTable.getEntry("tv");

        // Camera mounting from constants
        m_cameraHeight = k_frontCameraTransform.getZ();
        m_cameraPitch = k_frontCameraTransform.getRotation().getY();
        m_cameraForwardOffset = k_frontCameraTransform.getX();
    }

    /**
     * Gets all detected robot obstacles within range, converted to field coordinates.
     *
     * @return List of detected obstacles
     */
    public List<DetectedRobot> getDetectedRobots() {
        if (m_useSimMock && !m_simMockRobots.isEmpty()) {
            return new ArrayList<>(m_simMockRobots);
        }
        return new ArrayList<>(m_detectedRobots);
    }

    /**
     * Gets obstacles that are within a specific corridor.
     *
     * @param corridor The corridor to check
     * @return List of obstacles in that corridor
     */
    public List<DetectedRobot> getDetectedRobotsInCorridor(Corridor corridor) {
        return getDetectedRobots().stream()
            .filter(obs -> obs.isInCorridor(corridor))
            .toList();
    }

    /**
     * Checks if a specific corridor has any robot obstacles.
     *
     * @param corridor The corridor to check
     * @return true if corridor contains obstacles
     */
    public boolean isCorridorBlocked(Corridor corridor) {
        return getDetectedRobots().stream()
            .anyMatch(obs -> obs.isInCorridor(corridor));
    }

    /**
     * Updates the robot pose and recalculates obstacle field positions.
     * Should be called periodically with current robot pose.
     *
     * @param robotPose Current robot pose on the field
     */
    public void updateDetectedRobots(Pose2d robotPose) {
        m_robotPose = robotPose;
        parseDetections();
    }

    /**
     * Enables or disables simulation mock mode.
     *
     * @param enabled true to use mock obstacles, false to use real detections
     */
    public void setSimulationMode(boolean enabled) {
        m_useSimMock = enabled;
    }

    /**
     * Injects mock obstacles for simulation testing.
     *
     * @param obstacles List of mock obstacles
     */
    public void setSimMockObstacles(List<DetectedRobot> obstacles) {
        m_simMockRobots = new ArrayList<>(obstacles);
    }

    /**
     * Injects simulated robot positions as obstacles.
     *
     * @param robotPositions Array of robot positions on the field
     */
    public void injectSimulatedRobots(Translation2d[] robotPositions) {
        double timestamp = Timer.getFPGATimestamp();
        m_simMockRobots.clear();

        for (Translation2d pos : robotPositions) {
            double distance = m_robotPose.getTranslation().getDistance(pos);
            if (distance <= k_detectionRange && distance > 0.5) { // Don't detect self
                m_simMockRobots.add(new DetectedRobot(pos, 0.9, timestamp));
            }
        }
    }

    /**
     * Parses the Limelight JSON output and converts detections to field obstacles.
     * Uses regex parsing to avoid external JSON library dependencies.
     */
    private void parseDetections() {
        m_detectedRobots.clear();

        // Check if target is valid
        double tv = m_tvEntry.getDouble(0.0);
        if (tv < 0.5) {
            return; // No valid targets
        }

        String json = m_jsonEntry.getString("");
        if (json.isEmpty()) {
            return;
        }

        double timestamp = Timer.getFPGATimestamp();

        try {
            // Find all detection objects in the JSON
            Matcher matcher = DETECTOR_PATTERN.matcher(json);

            while (matcher.find()) {
                String className = matcher.group(1);
                double confidence = Double.parseDouble(matcher.group(2));
                double tx = Double.parseDouble(matcher.group(3));
                double ty = Double.parseDouble(matcher.group(4));

                // Only process robot/bumper detections
                if (!className.equalsIgnoreCase("robot") && !className.equalsIgnoreCase("bumper")) {
                    continue;
                }

                // Skip low confidence detections
                if (confidence < k_minConfidence) {
                    continue;
                }

                // Convert to field position
                Translation2d fieldPosition = cameraToField(tx, ty);

                // Check range
                double distance = m_robotPose.getTranslation().getDistance(fieldPosition);
                if (distance > k_detectionRange || distance < 0.3) {
                    continue;
                }

                m_detectedRobots.add(new DetectedRobot(fieldPosition, confidence, timestamp));
            }
        } catch (Exception e) {
            // Parsing failed, skip this frame
            SmartDashboard.putString("FrontLimelight/Error", e.getMessage());
        }
    }

    /**
     * Converts camera-relative angles to a field-relative position.
     * Uses trigonometry based on camera mounting position.
     *
     * @param tx Horizontal angle offset (degrees)
     * @param ty Vertical angle offset (degrees)
     * @return Field-relative position of the detected object
     */
    private Translation2d cameraToField(double tx, double ty) {
        // Convert angles to radians
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);

        // Estimate distance using vertical angle and known target height
        // Assuming we're detecting robot bumpers at approximately 0.2m height
        double targetHeight = 0.2;
        double angleToPose = m_cameraPitch + tyRad;

        double distance;
        if (Math.abs(angleToPose) < 0.01) {
            distance = k_detectionRange; // Avoid division by near-zero
        } else {
            distance = (m_cameraHeight - targetHeight) / Math.tan(-angleToPose);
        }

        // Clamp distance to reasonable range
        distance = Math.max(0.5, Math.min(distance, k_detectionRange));

        // Calculate robot-relative position
        double robotRelativeX = distance * Math.cos(txRad) + m_cameraForwardOffset;
        double robotRelativeY = distance * Math.sin(txRad);

        // Transform to field coordinates
        Translation2d robotRelative = new Translation2d(robotRelativeX, robotRelativeY);
        Translation2d fieldPosition = m_robotPose.getTranslation()
            .plus(robotRelative.rotateBy(m_robotPose.getRotation()));

        return fieldPosition;
    }

    /** Returns whether the Limelight has valid targets. */
    public boolean hasTargets() {
        return m_tvEntry.getDouble(0.0) >= 0.5;
    }

    @Override
    public void periodic() {
        // Parse detections if we have a valid robot pose
        if (m_robotPose.getTranslation().getNorm() > 0.01 && !m_useSimMock) {
            parseDetections();
        }

        // Telemetry
        SmartDashboard.putBoolean("FrontLimelight/HasTargets", hasTargets());
        SmartDashboard.putNumber("FrontLimelight/ObstacleCount", getDetectedRobots().size());
        SmartDashboard.putBoolean("FrontLimelight/SimMode", m_useSimMock);

        // Publish obstacle positions for visualization
        List<DetectedRobot> obstacles = getDetectedRobots();
        if (!obstacles.isEmpty()) {
            double[] obstacleData = new double[obstacles.size() * 2];
            for (int i = 0; i < obstacles.size(); i++) {
                DetectedRobot obs = obstacles.get(i);
                obstacleData[i * 2 + 0] = obs.position().getX();
                obstacleData[i * 2 + 1] = obs.position().getY();
            }
            SmartDashboard.putNumberArray("FrontLimelight/Obstacles", obstacleData);
        } else {
            SmartDashboard.putNumberArray("FrontLimelight/Obstacles", new double[0]);
        }
    }
}
