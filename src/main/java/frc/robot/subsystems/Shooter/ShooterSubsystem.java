package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/**
 * @brief Shooter subsystem for controlling the flywheel.
 *
 * <p>Manages the shooter flywheel motor and provides both velocity control
 * and open-loop voltage control modes.
 */
public class ShooterSubsystem extends SubsystemBase {
    
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private double targetVelocityRPM = 0.0;
    private boolean velocityMode = false;
    
    private final Debouncer atTargetDebouncer = new Debouncer(kAtTargetDebounceTime);
    
    // Tunable shooter RPM value that can be adjusted from NetworkTables/AdvantageScope
    private final LoggedNetworkNumber tunableShooterRPM = 
        new LoggedNetworkNumber("/Tuning/Shooter/TargetRPM", 3500.0);

    // Tunable hood pitch angle for table characterization
    private final LoggedNetworkNumber tunableHoodPitchDeg =
        new LoggedNetworkNumber("/Tuning/Shooter/HoodPitchDeg", 20.0);
    
    // Tunable PID values
    private final LoggedNetworkNumber tunableKP = 
        new LoggedNetworkNumber("/Tuning/Shooter/kP", kP);
    private final LoggedNetworkNumber tunableKI = 
        new LoggedNetworkNumber("/Tuning/Shooter/kI", kI);
    private final LoggedNetworkNumber tunableKD = 
        new LoggedNetworkNumber("/Tuning/Shooter/kD", kD);
    
    // Tunable feedforward values
    private final LoggedNetworkNumber tunableKS = 
        new LoggedNetworkNumber("/Tuning/Shooter/kS", kS);
    private final LoggedNetworkNumber tunableKV = 
        new LoggedNetworkNumber("/Tuning/Shooter/kV", kV);
    private final LoggedNetworkNumber tunableKA = 
        new LoggedNetworkNumber("/Tuning/Shooter/kA", kA);
    
    private double lastKP = kP;
    private double lastKI = kI;
    private double lastKD = kD;
    private double lastKS = kS;
    private double lastKV = kV;
    private double lastKA = kA;

    /**
     * @brief Create a shooter subsystem.
     * @param io Hardware interface implementation
     */
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        
        // Check if PID values have changed and update if needed
        double currentKP = tunableKP.get();
        double currentKI = tunableKI.get();
        double currentKD = tunableKD.get();
        if (currentKP != lastKP || currentKI != lastKI || currentKD != lastKD) {
            io.setPID(currentKP, currentKI, currentKD);
            lastKP = currentKP;
            lastKI = currentKI;
            lastKD = currentKD;
            Logger.recordOutput("Shooter/PIDUpdated", true);
        }
        
        // Check if FF values have changed and update if needed
        double currentKS = tunableKS.get();
        double currentKV = tunableKV.get();
        double currentKA = tunableKA.get();
        if (currentKS != lastKS || currentKV != lastKV || currentKA != lastKA) {
            io.setFF(currentKS, currentKV, currentKA);
            lastKS = currentKS;
            lastKV = currentKV;
            lastKA = currentKA;
            Logger.recordOutput("Shooter/FFUpdated", true);
        }
        
        Logger.recordOutput("Shooter/TargetVelocityRPM", targetVelocityRPM);
        Logger.recordOutput("Shooter/VelocityMode", velocityMode);
        Logger.recordOutput("Shooter/AtTarget", isAtTarget());
        Logger.recordOutput("Shooter/VelocityErrorRPM", targetVelocityRPM - inputs.flywheelVelocityRPM);
        Logger.recordOutput("Shooter/FollowerErrorRPM", inputs.flywheelVelocityRPM - inputs.followerVelocityRPM);
    }

    /**
     * @brief Set shooter velocity in RPM (closed-loop).
     * @param velocityRPM Target velocity in RPM
     */
    public void setVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        velocityMode = true;
        io.setVelocity(velocityRPM);
    }

    /**
     * @brief Set shooter voltage (open-loop).
     * @param volts Voltage to apply (-12 to 12)
     */
    public void setVoltage(double volts) {
        velocityMode = false;
        targetVelocityRPM = 0.0;
        io.setVoltage(volts);
    }

    /**
     * @brief Stop the shooter.
     */
    public void stop() {
        velocityMode = false;
        targetVelocityRPM = 0.0;
        io.stop();
    }

    /**
     * @brief Check if shooter is at target velocity (with debouncing).
     * @return true if within tolerance for debounce period
     */
    public boolean isAtTarget() {
        if (!velocityMode) {
            return false;
        }
        double error = Math.abs(targetVelocityRPM - inputs.flywheelVelocityRPM);
        return atTargetDebouncer.calculate(error < kVelocityToleranceRPM);
    }

    /**
     * @brief Get current flywheel velocity in RPM.
     * @return Current velocity
     */
    public double getVelocityRPM() {
        return inputs.flywheelVelocityRPM;
    }

    /**
     * @brief Get target flywheel velocity in RPM.
     * @return Target velocity
     */
    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

    /**
     * @brief Get the tunable shooter RPM from NetworkTables.
     * @return Tunable RPM value
     */
    public double getTunableRPM() {
        return tunableShooterRPM.get();
    }

    /**
     * @brief Get the tunable hood pitch angle from NetworkTables.
     * @return Tunable pitch angle in degrees (up from horizontal)
     */
    public double getTunablePitchDegrees() {
        return tunableHoodPitchDeg.get();
    }

    /**
     * @brief Set shooter to the tunable RPM value from NetworkTables.
     */
    public void setTunableVelocity() {
        setVelocity(tunableShooterRPM.get());
    }

    /**
     * @brief Check if shooter is running in velocity mode.
     * @return true if velocity control is active
     */
    public boolean isVelocityMode() {
        return velocityMode;
    }

    /**
     * @brief Check if shooter hardware is connected.
     * @return true if connected
     */
    public boolean isConnected() {
        return inputs.connected;
    }

    /**
     * @brief Set PID gains for velocity control.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    /**
     * @brief Set feedforward gains for velocity control.
     * @param kS Static friction (V)
     * @param kV Velocity feedforward (V/(rad/s))
     * @param kA Acceleration feedforward (V/(rad/s^2))
     */
    public void setFF(double kS, double kV, double kA) {
        io.setFF(kS, kV, kA);
    }
     
    /**
     * Run characterization with specified voltage.
     */
    public void runCharacterization(double volts) {
        setVoltage(volts);
    }

    /**
     * Get current velocity in rotations/second at motor shaft for characterization.
     */
    public double getCharacterizationVelocity() {
        return (inputs.flywheelVelocityRPM / 60.0) * kGearRatio;
    }
}

