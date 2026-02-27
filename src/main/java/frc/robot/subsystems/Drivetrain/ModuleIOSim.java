package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.ModuleConstants;

/**
 * Physics sim implementation of module IO.
 * Mimics ModuleIOTalonFX behavior exactly:
 * - Drive: Receives motor shaft rad/s, reports wheel rad/s
 * - Turn: Uses gear ratio internally, reports mechanism position
 */
public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    
    private final PIDController driveController = new PIDController(kDriveSimP, 0, kDriveSimD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    
    // Turn control
    private final PIDController turnController = new PIDController(kTurnSimP, 0, kTurnSimD);
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        // Drive sim: Model the motor directly (gear ratio 1.0), then handle gearing in our code
        // This way the motor physics are correct (628 rad/s free speed at 12V)
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    kDriveGearbox,
                    0.001,  // Low MOI for responsive sim
                    1.0),   // Model motor shaft directly, no gear ratio in physics
                kDriveGearbox);
        
        // Turn sim: gear ratio causes DCMotorSim to output mechanism values
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    kTurnGearbox,
                    0.004,
                    ModuleConstants.kTurningMotorGearRatio),
                kTurnGearbox);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update sims first
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Calculate control voltages AFTER update (for next cycle)
        if (driveClosedLoop) {
            // Combine feedforward + feedback on motor shaft velocity
            double feedbackVolts = driveController.calculate(driveSim.getAngularVelocityRadPerSec());
            double totalVolts = driveFFVolts + feedbackVolts;
            driveAppliedVolts = MathUtil.clamp(totalVolts, -12.0, 12.0);
            
            // Debug - occasional console print
            if (Math.random() < 0.1) { // 10% chance to print
                System.out.println("=== ModuleIOSim updateInputs ===");
                System.out.println("Motor shaft velocity: " + driveSim.getAngularVelocityRadPerSec() + " rad/s");
                System.out.println("Motor shaft setpoint: " + driveController.getSetpoint() + " rad/s");
                System.out.println("FF Volts: " + driveFFVolts);
                System.out.println("Feedback Volts: " + feedbackVolts);
                System.out.println("Total Volts: " + totalVolts);
                System.out.println("Applied Volts (clamped): " + driveAppliedVolts);
                System.out.println("================================");
            }
        } else {
            driveController.reset();
        }
        
        if (turnClosedLoop) {
            // Feedback on mechanism position (what turnSim outputs)
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
            turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
        } else {
            turnController.reset();
        }

        // Apply voltages for next cycle
        driveSim.setInputVoltage(driveAppliedVolts);
        turnSim.setInputVoltage(turnAppliedVolts);

        // Report drive values (convert motor shaft to wheel, like TalonFX)
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad() / ModuleConstants.kDriveMotorGearRatio;
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec() / ModuleConstants.kDriveMotorGearRatio;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Report turn values (already mechanism values, like TalonFX with RotorToSensorRatio)
        inputs.turnConnected = true;
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        // Odometry
        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public void setDriveOpenLoop(double volts) {
        driveClosedLoop = false;
        driveAppliedVolts = volts;
    }

    @Override
    public void setTurnOpenLoop(double volts) {
        turnClosedLoop = false;
        turnAppliedVolts = volts;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        
        // velocityRadPerSec is motor shaft rad/s from Module.java
        // kDriveSimKv is characterized for WHEEL rad/s (velocity range 1-4.27 rad/s confirms this)
        // Convert motor shaft to wheel velocity
        double wheelVelocityRadPerSec = velocityRadPerSec / ModuleConstants.kDriveMotorGearRatio;
        driveFFVolts = kDriveSimKs * Math.signum(wheelVelocityRadPerSec) + kDriveSimKv * wheelVelocityRadPerSec;
        
        // Debug - print once to console
        if (Math.abs(velocityRadPerSec) > 100) {
            System.out.println("=== ModuleIOSim setDriveVelocity ===");
            System.out.println("Motor shaft setpoint: " + velocityRadPerSec + " rad/s");
            System.out.println("Wheel velocity: " + wheelVelocityRadPerSec + " rad/s");
            System.out.println("kDriveSimKs: " + kDriveSimKs);
            System.out.println("kDriveSimKv: " + kDriveSimKv);
            System.out.println("FF Volts: " + driveFFVolts);
            System.out.println("=====================================");
        }
        
        // Set PID setpoint in motor shaft rad/s (what driveSim outputs)
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setTurningPID(double kp, double ki, double kd) {
        turnController.setP(kp);
        turnController.setI(ki);
        turnController.setD(kd);
    }
}
