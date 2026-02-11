// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.PathfindingConstants.k_basinCenter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.commands.Autos;
import frc.robot.commands.CycleCommands;
import frc.robot.simulation.SimulatedGameState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.utils.BlackBox;
import frc.robot.utils.Telemetry;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class RobotContainer {

    // Override the arena BEFORE any subsystems are created
    static {
        if (RobotBase.isSimulation()) {
            SimulatedArena.overrideInstance(new Arena2026Rebuilt());
        }
    }

    // Controllers
    private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);

    // Subsystems
    private final Swerve m_swerve = new Swerve();
    private final Vision m_vision = new Vision();
    private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

    // Simulation
    private SimulatedGameState m_gameState = null;

    // Auto chooser
    private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

    public RobotContainer() {
        // Configure pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        m_swerve.runAutoBuilder();

        // Configure autos
        m_autochooser.setDefaultOption("square", Autos.simpleSquareAuto(m_swerve, m_vision));
        m_autochooser.addOption("drive under tag 28", Autos.driveUnderTagAuto(m_swerve, 28));
        m_autochooser.addOption("autoalign reef A", Autos.autoAlignReef(m_swerve, 18));
        SmartDashboard.putData(m_autochooser);

        // Default commands
        m_swerve.setDefaultCommand(swerveDefaultCommand());
        m_vision.setDefaultCommand(visionDefaultCommand());

        // Initialize simulation
        if (RobotBase.isSimulation()) {
            m_gameState = new SimulatedGameState(m_swerve::getSimPose, m_swerve::getFieldSpeeds);
            m_gameState.resetFieldForAuto();
        }

        configureBindings();
    }

    private void configureBindings() {
        m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));
        m_joystick.start().whileTrue(run(() -> BlackBox.DataRecorder.recordData("heading", m_swerve.getGyroHeading())));
        m_joystick.a().onTrue(CycleCommands.createCycleCommand(m_swerve, m_vision, this::hasManualDriveInput));

        if (RobotBase.isSimulation() && m_gameState != null) {
            m_joystick.rightBumper().onTrue(runOnce(() -> m_gameState.shootFuel()));
            m_joystick.leftBumper().onTrue(runOnce(() -> m_gameState.resetFieldForAuto()));
            // B button: Rapid fire 10 shots from turret with calculated velocity
            m_joystick.b().onTrue(CycleCommands.createRapidFireCommand(
                m_gameState,
                () -> m_vision.getTurretCamera().getTurretYaw()
            ));
        }
    }

    private boolean hasManualDriveInput() {
        final double deadband = 0.2;
        return Math.abs(m_joystick.getLeftX()) > deadband
            || Math.abs(m_joystick.getLeftY()) > deadband
            || Math.abs(m_joystick.getRightX()) > deadband;
    }

    private ParallelCommandGroup visionDefaultCommand() {
        ParallelCommandGroup cmd = new ParallelCommandGroup();
        if (RobotBase.isSimulation())
            cmd.addCommands(run(() -> m_vision.updatePose(m_swerve.getSimPose())));
        cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
        cmd.addRequirements(m_vision);
        return cmd;
    }

    private Command swerveDefaultCommand() {
        Command driveCommand = new RunCommand(
            () -> m_swerve.drive(new ChassisSpeeds(
                MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
            m_swerve);

        Command turretTrackingCommand = run(() -> {
            m_vision.getTurretCamera().aimAtFieldPose(m_swerve.getPose(), k_basinCenter);
        });

        return new ParallelCommandGroup(driveCommand, turretTrackingCommand);
    }

    public Command getAutonomousCommand() {
        return m_autochooser.getSelected();
    }

    /** Called in robotPeriodic for simulation updates. */
    public void testRun() {
        if (RobotBase.isSimulation() && m_gameState != null) {
            m_gameState.update();

            Pose3d[] fuel = m_gameState.getFuelPoses();
            double[] fuelData = new double[fuel.length * 7];
            for (int i = 0; i < fuel.length; i++) {
                Pose3d f = fuel[i];
                fuelData[i * 7 + 0] = f.getX();
                fuelData[i * 7 + 1] = f.getY();
                fuelData[i * 7 + 2] = f.getZ();
                fuelData[i * 7 + 3] = f.getRotation().getQuaternion().getW();
                fuelData[i * 7 + 4] = f.getRotation().getQuaternion().getX();
                fuelData[i * 7 + 5] = f.getRotation().getQuaternion().getY();
                fuelData[i * 7 + 6] = f.getRotation().getQuaternion().getZ();
            }
            SmartDashboard.putNumberArray("Sim/FuelPoses", fuelData);
            SmartDashboard.putNumber("Sim/FuelCount", fuel.length);
        }
    }
}
