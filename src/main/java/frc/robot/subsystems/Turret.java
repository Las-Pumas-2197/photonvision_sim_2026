// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem that aims at field positions using odometry-based localization.
 * Uses robot pose to calculate turret angle - no camera vision tracking.
 *
 * Ready for hardware motor implementation.
 */
public class Turret extends SubsystemBase {

  // Turret state - initialize to 180° (back of robot, center of valid range)
  private double m_targetYaw = Math.PI;      // Target yaw angle (radians)
  private double m_currentYaw = Math.PI;     // Current yaw angle (radians)

  // Turret physical limits (radians)
  // Range: 135° to 270° (-90°) - covers left-rear, back, and right of robot
  // Inside angle passes through 180° (back of robot), does NOT include 0° (front)
  private static final double MIN_YAW = Math.toRadians(135);   // 135 degrees (left-rear)
  private static final double MAX_YAW = Math.toRadians(-90);   // 270 degrees (right)

  // TODO: Add motor controller
  // private final TalonFX m_turretMotor;
  // private final CANcoder m_turretEncoder;

  // TODO: Add PID controller for position control
  // private final PIDController m_pidController;

  public Turret() {
    // TODO: Initialize motor controller
    // m_turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    // m_turretEncoder = new CANcoder(TurretConstants.ENCODER_ID);

    // TODO: Configure motor
    // configureTurretMotor();
  }

  // ===== Turret Control Methods =====

  /**
   * Sets the target turret yaw angle.
   * @param yawRadians Target yaw in radians (relative to robot heading)
   */
  public void setTargetYaw(double yawRadians) {
    // Clamp to physical limits
    m_targetYaw = clampYaw(yawRadians);
  }

  /**
   * Gets the target turret yaw angle.
   */
  public double getTargetYaw() {
    return m_targetYaw;
  }

  /**
   * Gets the current turret yaw angle.
   * In simulation, this returns the target. With hardware, read from encoder.
   */
  public double getCurrentYaw() {
    // TODO: Read from encoder when hardware is implemented
    // return m_turretEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    return m_currentYaw;
  }

  /**
   * Returns true if the turret is at the target position (within tolerance).
   */
  public boolean isAtTarget() {
    return Math.abs(m_currentYaw - m_targetYaw) < Math.toRadians(2.0);
  }

  /**
   * Stops the turret motor.
   */
  public void stop() {
    // TODO: Stop motor when hardware is implemented
    // m_turretMotor.stopMotor();
  }

  // ===== Aiming Methods (Field Localization) =====

  /**
   * Calculates the angle to aim at a fixed field position.
   * @param robotPose The current robot pose from odometry
   * @param targetPose The field position to aim at
   * @return The yaw angle in radians to aim at the target
   */
  public double calculateAngleToFieldPose(Pose2d robotPose, Pose2d targetPose) {
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double targetX = targetPose.getX();
    double targetY = targetPose.getY();

    double angleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
    double robotHeading = robotPose.getRotation().getRadians();
    double turretAngle = angleToTarget - robotHeading;

    return normalizeAngle(turretAngle);
  }

  /**
   * Aims the turret at a fixed field position.
   * @param robotPose The current robot pose from odometry
   * @param targetPose The field position to aim at
   */
  public void aimAtFieldPose(Pose2d robotPose, Pose2d targetPose) {
    setTargetYaw(calculateAngleToFieldPose(robotPose, targetPose));
  }

  /**
   * Calculates the lead-corrected angle to aim at a field position while moving.
   * Uses iterative refinement to converge on the correct lead amount.
   *
   * @param robotPose The current robot pose
   * @param targetPose The field position to aim at
   * @param fieldRelativeSpeeds The robot's field-relative velocity
   * @param exitVelocity The projectile exit velocity (m/s)
   * @param launchAngle The projectile launch angle above horizontal (radians)
   * @return The lead-corrected turret yaw angle in radians
   */
  public double calculateLeadCorrectedAngle(
      Pose2d robotPose,
      Pose2d targetPose,
      ChassisSpeeds fieldRelativeSpeeds,
      double exitVelocity,
      double launchAngle) {

    Translation2d robotPos = robotPose.getTranslation();
    Translation2d targetPos = targetPose.getTranslation();
    double horizontalVelocity = exitVelocity * Math.cos(launchAngle);

    // Iterative lead correction - 3 iterations for convergence
    Translation2d adjustedTarget = targetPos;
    for (int i = 0; i < 3; i++) {
      double distance = robotPos.getDistance(adjustedTarget);
      double timeOfFlight = distance / horizontalVelocity;

      // Calculate drift due to robot velocity during flight
      double driftX = fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight;
      double driftY = fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight;

      // Adjust aim point to compensate for drift
      adjustedTarget = new Translation2d(
          targetPos.getX() - driftX,
          targetPos.getY() - driftY
      );
    }

    // Calculate angle to final adjusted target
    double angleToTarget = Math.atan2(
        adjustedTarget.getY() - robotPos.getY(),
        adjustedTarget.getX() - robotPos.getX()
    );
    double robotHeading = robotPose.getRotation().getRadians();
    double turretAngle = angleToTarget - robotHeading;

    return normalizeAngle(turretAngle);
  }

  /**
   * Aims the turret at a field position with lead correction for robot movement.
   * @param robotPose The current robot pose
   * @param targetPose The field position to aim at
   * @param fieldRelativeSpeeds The robot's field-relative velocity
   * @param exitVelocity The projectile exit velocity (m/s)
   * @param launchAngle The projectile launch angle above horizontal (radians)
   */
  public void aimAtFieldPoseWithLead(
      Pose2d robotPose,
      Pose2d targetPose,
      ChassisSpeeds fieldRelativeSpeeds,
      double exitVelocity,
      double launchAngle) {
    setTargetYaw(calculateLeadCorrectedAngle(robotPose, targetPose, fieldRelativeSpeeds, exitVelocity, launchAngle));
  }

  // ===== Utility Methods =====

  /**
   * Normalizes an angle to the range [-PI, PI].
   */
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }

  /**
   * Clamps yaw to physical turret limits.
   * Range: 135° to 270° (-90°) through the back (180°)
   */
  private double clampYaw(double yaw) {
    // First normalize to -PI to PI
    yaw = normalizeAngle(yaw);

    // Range wraps around ±180°, so check if already in valid range
    if (isYawInRange(yaw)) {
      return yaw;
    }

    // Find closest limit
    double distToMin = Math.abs(normalizeAngle(yaw - MIN_YAW));
    double distToMax = Math.abs(normalizeAngle(yaw - MAX_YAW));
    return distToMin < distToMax ? MIN_YAW : MAX_YAW;
  }

  /**
   * Returns true if the given yaw angle is within the turret's physical range.
   * Range: 135° to 270° (-90°) through the back (180°)
   */
  public boolean isYawReachable(double yaw) {
    return isYawInRange(normalizeAngle(yaw));
  }

  /**
   * Helper to check if yaw is in the valid range (135° to -90° through 180°).
   */
  private boolean isYawInRange(double yaw) {
    // Valid range: yaw >= 135° OR yaw <= -90° (wraps through ±180°)
    return yaw >= MIN_YAW || yaw <= MAX_YAW;
  }

  /**
   * Returns true if the turret is currently at one of its physical limits.
   */
  public boolean isAtLimit() {
    final double tolerance = Math.toRadians(2.0);
    return Math.abs(m_currentYaw - MIN_YAW) < tolerance
        || Math.abs(m_currentYaw - MAX_YAW) < tolerance;
  }

  // ===== Simulation Support =====

  /**
   * Updates the turret simulation. Call this in simulation periodic.
   * Simulates motor movement toward target position.
   */
  public void updateSimulation(double dt) {
    // Simple simulation: move toward target at a max rate
    double maxRate = Math.toRadians(360); // 360 deg/sec max rotation
    double error = m_targetYaw - m_currentYaw;
    double maxDelta = maxRate * dt;

    if (Math.abs(error) < maxDelta) {
      m_currentYaw = m_targetYaw;
    } else {
      m_currentYaw += Math.signum(error) * maxDelta;
    }
  }

  /**
   * Sets the current yaw directly (for simulation initialization).
   */
  public void setCurrentYaw(double yawRadians) {
    m_currentYaw = yawRadians;
  }

  @Override
  public void periodic() {
    // TODO: When hardware is implemented, run PID control here
    // double output = m_pidController.calculate(getCurrentYaw(), m_targetYaw);
    // m_turretMotor.set(output);

    // Publish telemetry
    SmartDashboard.putNumber("Turret/TargetYaw", Math.toDegrees(m_targetYaw));
    SmartDashboard.putNumber("Turret/CurrentYaw", Math.toDegrees(m_currentYaw));
    SmartDashboard.putBoolean("Turret/AtTarget", isAtTarget());
    SmartDashboard.putBoolean("Turret/AtLimit", isAtLimit());
    SmartDashboard.putNumber("Turret/MinYaw", Math.toDegrees(MIN_YAW));
    SmartDashboard.putNumber("Turret/MaxYaw", Math.toDegrees(MAX_YAW));
  }
}
