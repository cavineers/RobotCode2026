package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {
    private enum ControlMode {
        DISABLED,
        MANUAL,
        POSITION
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final Supplier<Pose3d> robotPoseSupplier;

    private ControlMode controlMode = ControlMode.DISABLED;

    private double manualDemandVolts = 0.0;

    @AutoLogOutput(key = "Turret/CommandedFieldAngleRad")
    private double commandedFieldAngleRad = Double.NaN;

    @AutoLogOutput(key = "Turret/CommandedTurretAngleRad")
    private double commandedTurretAngleRad = Double.NaN;

    @AutoLogOutput(key = "Turret/ClosedLoopEnabled")
    private boolean closedLoopEnabled = true;

    @AutoLogOutput(key = "Turret/Homed")
    private boolean homed = !TurretConstants.kUseHomingSwitch;

    private final LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Turret/PositionKp", TurretConstants.kPositionKp);
    private final LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Turret/PositionKi", TurretConstants.kPositionKi);
    private final LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Turret/PositionKd", TurretConstants.kPositionKd);
    private double currentKp = TurretConstants.kPositionKp;
    private double currentKi = TurretConstants.kPositionKi;
    private double currentKd = TurretConstants.kPositionKd;
    
    private boolean lastHomeSwitchState = false;


    public Turret(TurretIO io, Supplier<Pose3d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public Turret(TurretIO io) {
        this(io, () -> new Pose3d());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        updateTunableGains();

        Logger.processInputs("Turret", inputs);

        handleHoming();

        switch (controlMode) {
            case POSITION -> runClosedLoop();
            case MANUAL -> runManual();
            case DISABLED -> stopOutputs();
        }

        Logger.recordOutput("Turret/ControlMode", controlMode.name());
        Logger.recordOutput("Turret/ManualDemandVolts", controlMode == ControlMode.MANUAL ? manualDemandVolts : 0.0);
        Logger.recordOutput("Turret/FieldAngleRad", getCurrentFieldAngleRad());
        Logger.recordOutput("Turret/PositionErrorRad", getPositionError());
        
        Pose3d robotPose = robotPoseSupplier.get();
        // Since Rotation3d(x, y, z) is roll-pitch-yaw, using z-rotation for yaw
        // Also manually adding robot pose rotation since the previous getZ() was returning 0.0 or failing
        
      
        Pose3d turretPose = new Pose3d(
            robotPose.getTranslation().plus(new Translation3d(0.0, 0.0, 0.5
            )), 
            new Rotation3d(0.0, 0.0, getCurrentFieldAngleRad())
        );
        
        Logger.recordOutput("Turret/FieldPose3d", turretPose);
    }

    public void setFieldRelativeTarget(double fieldAngleRad) {
        commandedFieldAngleRad = wrapAngle(fieldAngleRad);
        controlMode = ControlMode.POSITION;
    }

    public void setRobotRelativeTarget(double turretAngleRad) {
        double robotHeading = wrapAngle(robotPoseSupplier.get().getRotation().getZ());
        double normalizedTurret = wrapAngle(turretAngleRad);
        setFieldRelativeTarget(normalizedTurret + robotHeading);
    }

    public void holdCurrentPosition() {
        commandedFieldAngleRad = getCurrentFieldAngleRad();
        controlMode = ControlMode.POSITION;
    }

    public void setManualVoltage(double volts) {
        // If not homed, limit to slow homing search voltage for safety
        double maxAllowedVoltage = homed ? TurretConstants.kMaxVoltage : TurretConstants.kHomingSearchMaxVoltage;
        manualDemandVolts = MathUtil.clamp(volts, -maxAllowedVoltage, maxAllowedVoltage);
        controlMode = ControlMode.MANUAL;
    }

    public void stop() {
        controlMode = ControlMode.DISABLED;
        manualDemandVolts = 0.0;
    }

    public void resetEncoder(double positionRad) {
        io.resetEncoder(positionRad);
    }

    public void enableClosedLoop(boolean enable) {
        closedLoopEnabled = enable;
        if (!enable) {
            controlMode = ControlMode.MANUAL;
        }
    }

    @AutoLogOutput(key = "Turret/AtTarget")
    public boolean isAtTarget() {
        if (controlMode != ControlMode.POSITION || !hasValidTarget()) {
            return false;
        }

        double positionError = Math.abs(commandedTurretAngleRad - inputs.positionRad);
        boolean positionOk = positionError <= TurretConstants.kPositionToleranceRad;
        boolean velocityOk = Math.abs(inputs.velocityRadPerSec) <= TurretConstants.kVelocityToleranceRadPerSec;
        return positionOk && velocityOk;
    }

    public double getCurrentTurretAngleRad() {
        return inputs.positionRad;
    }

    public double getCurrentFieldAngleRad() {
        return wrapAngle(inputs.positionRad + TurretConstants.kTurretZeroOffsetRad + robotPoseSupplier.get().getRotation().getZ());
    }

    public double getTargetFieldAngleRad() {
        return hasValidTarget() ? commandedFieldAngleRad : Double.NaN;
    }

    public boolean isHomed() {
        return homed;
    }

    public double getPositionError() {
        if (!hasValidTarget()) {
            return Double.NaN;
        }
        return commandedTurretAngleRad - inputs.positionRad;
    }

    @AutoLogOutput(key = "Turret/TargetLocked")
    public boolean isTargetLocked() {
        if (!homed || controlMode != ControlMode.POSITION || !hasValidTarget()) {
            return false;
        }
        
        // Check if the commanded target had to be clamped
        double robotHeading = wrapAngle(robotPoseSupplier.get().getRotation().getZ());
        double desiredRobotRelative = wrapAngle(commandedFieldAngleRad - robotHeading - TurretConstants.kTurretZeroOffsetRad);
        
        // If desired angle is within our physical limits, we can lock on
        boolean targetReachable = desiredRobotRelative >= TurretConstants.kMinAngleRad && 
                                  desiredRobotRelative <= TurretConstants.kMaxAngleRad;
        
        // And we're close enough to the (possibly clamped) position
        boolean atPosition = isAtTarget();
        
        return targetReachable && atPosition;
    }

    private void runClosedLoop() {
        if (!hasValidTarget()) {
            holdCurrentPosition();
        }

        commandedTurretAngleRad = calculateRobotRelativeSetpoint(commandedFieldAngleRad);

        if (closedLoopEnabled) {
            io.setPositionSetpoint(commandedTurretAngleRad);
        } else {
            io.setVoltage(applySoftLimits(0.0));
        }
    }

    private void runManual() {
        io.setVoltage(applySoftLimits(manualDemandVolts));
    }

    private void stopOutputs() {
        io.stop();
    }

    private void handleHoming() {
        if (!TurretConstants.kUseHomingSwitch) {
            return;
        }

        boolean pressed = inputs.zeroSwitchPressed;

        // Home on rising edge OR if switch is pressed on first run (robot booted while on switch)
        if (pressed && (!lastHomeSwitchState || !homed)) {
            io.resetEncoder(TurretConstants.kHomingSwitchZeroPositionRad);
            homed = true;
            commandedFieldAngleRad = getCurrentFieldAngleRad();
            commandedTurretAngleRad = 0.0;
        }

        lastHomeSwitchState = pressed;
    }

    private double calculateRobotRelativeSetpoint(double fieldAngleRad) {
        double normalizedField = wrapAngle(fieldAngleRad);
        double robotHeading = wrapAngle(robotPoseSupplier.get().getRotation().getZ());
        
        // Debug components
        Logger.recordOutput("Turret/Debug/Setpt/Field", normalizedField);
        Logger.recordOutput("Turret/Debug/Setpt/Heading", robotHeading);
        
        // To find the necessary turret angle:
        // FieldTarget = RobotHeading + TurretOffset + TurretAngle
        // TurretAngle = FieldTarget - RobotHeading - TurretOffset
        double rawRel = normalizedField - robotHeading - TurretConstants.kTurretZeroOffsetRad;
        Logger.recordOutput("Turret/Debug/Setpt/RawRel", rawRel);

        double robotRelative = wrapAngle(rawRel);
        Logger.recordOutput("Turret/Debug/Setpt/WrappedRel", robotRelative);

        double clamped = MathUtil.clamp(robotRelative, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
        Logger.recordOutput("Turret/Debug/Setpt/Clamped", clamped);

        return clamped;
    }

    private double applySoftLimits(double volts) {
        double limited = volts;
        if (limited > 0.0 && inputs.forwardLimit) {
            limited = 0.0;
        } else if (limited < 0.0 && inputs.reverseLimit) {
            limited = 0.0;
        }
        return limited;
    }

    private boolean hasValidTarget() {
        return !Double.isNaN(commandedFieldAngleRad);
    }

    private static double wrapAngle(double angleRad) {
        // Normalize angle to the standard (-PI, +PI] range.
        // Mechanical/allowed turret limits are enforced separately using kMinAngleRad and kMaxAngleRad.
        // Using angleModulus avoids needing custom wrapping logic for 0-2PI (0-360 deg) ranges.
        return MathUtil.angleModulus(angleRad);
    }

    private void updateTunableGains() {
        double newKp = tuningP.get();
        double newKi = tuningI.get();
        double newKd = tuningD.get();
   
        boolean pChanged = Math.abs(newKp - currentKp) > 1e-4;
        boolean iChanged = Math.abs(newKi - currentKi) > 1e-4;
        boolean dChanged = Math.abs(newKd - currentKd) > 1e-4;

        if (pChanged || iChanged || dChanged) {
            currentKp = newKp;
            currentKi = newKi;
            currentKd = newKd;
            io.configureClosedLoop(currentKp, currentKi, currentKd);
          
        }
        
    }

}
