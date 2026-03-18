// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.ShotSolver;
import edu.wpi.first.wpilibj.Timer;


/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

    private final RobotContainer m_robotContainer;

    /** How often to wipe all FuelSim particles (seconds). Keeps the field readable. */
    private static final double FUEL_CLEAR_INTERVAL_SEC = 20.0;
    private final Timer fuelClearTimer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                        // be added.
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        m_robotContainer = new RobotContainer();

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.

        ShotSolver.solveSimple(m_robotContainer.drivetrain.getPose());
        
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // Home the turret at the start of autonomous, wait for it to complete, then run the auto command
        Command autonomousCommand = Commands.runOnce(() -> m_robotContainer.turret.startHoming())
            .andThen(Commands.waitUntil(() -> m_robotContainer.turret.isHomed()))
            .andThen(Commands.deferredProxy(() -> m_robotContainer.getAutonomousCommand()))
            .finallyDo(() -> {
                // Stop shooter and intake at end of autonomous
                m_robotContainer.shooter.stop();
                m_robotContainer.inBumperIntake.stopIntake();
            });

        // schedule the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        CommandScheduler.getInstance().cancelAll();
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        fuelClearTimer.start();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        if (m_robotContainer.fuelSim != null) {
            m_robotContainer.fuelSim.updateSim();

            if (fuelClearTimer.advanceIfElapsed(FUEL_CLEAR_INTERVAL_SEC)) {
                m_robotContainer.fuelSim.clearFuel();
            }
        }
    }
}
