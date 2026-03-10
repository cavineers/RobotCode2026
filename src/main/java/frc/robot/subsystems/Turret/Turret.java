package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {
    private enum ControlMode {
        DISABLED,
        MANUAL,
        POSITION,
        HOMING
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

    private final LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Turret/PositionKp", TurretConstants.kPositionKp);
    private final LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Turret/PositionKi", TurretConstants.kPositionKi);
    private final LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Turret/PositionKd", TurretConstants.kPositionKd);
    private double currentKp = TurretConstants.kPositionKp;
    private double currentKi = TurretConstants.kPositionKi;
    private double currentKd = TurretConstants.kPositionKd;

    // Homing state variables
    @AutoLogOutput(key = "Turret/Homed")
    private boolean homed = false;
    private int homingCurrentSpikeCount = 0;

    // Turret angle history buffer for moving camera support
    // Stores turret angles over time to allow retrospective camera position calculation
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(TurretConstants.kTurretAngleBufferSizeSec);


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

        // Add current turret angle to history buffer for moving camera support
        turretAngleBuffer.addSample(Timer.getFPGATimestamp(), new Rotation2d(getCurrentTurretAngleRad()));


        switch (controlMode) {
            case POSITION -> runClosedLoop();
            case MANUAL -> runManual();
            case HOMING -> runHoming();
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

    public void startHoming() {
        homed = false;
        controlMode = ControlMode.HOMING;
        homingCurrentSpikeCount = 0;
        Logger.recordOutput("Turret/HomingStarted", true);
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

    @AutoLogOutput(key="Turret/RobotRelativeAngle")
    public double getCurrentTurretAngleRad() {
        return inputs.positionRad;
    }

    public double getCurrentFieldAngleRad() {
        return wrapAngle(inputs.positionRad + TurretConstants.kTurretZeroOffsetRad + robotPoseSupplier.get().getRotation().getZ());
    }

    public double getTargetFieldAngleRad() {
        return hasValidTarget() ? commandedFieldAngleRad : Double.NaN;
    }

    public double getTargetTurretAngleRad() {
        return hasValidTarget() ? commandedTurretAngleRad : getCurrentTurretAngleRad();
    }

    public boolean isHomed() {
        return homed;
    }

    /**
     * Get the turret angle at a specific timestamp from the history buffer.
     * Used for calculating moving camera position when processing vision measurements.
     * @param timestamp The timestamp to query
     * @return The turret angle at that time, or current angle if timestamp not in buffer
     */
    public Rotation2d getTurretAngleAtTime(double timestamp) {
        return turretAngleBuffer.getSample(timestamp).orElse(new Rotation2d(getCurrentTurretAngleRad()));
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

    private void runHoming() {
        // Check if current is above threshold
        boolean currentSpikeDetected = inputs.supplyCurrentAmps >= TurretConstants.kHomingCurrentThresholdAmps;
        
        if (currentSpikeDetected) {
            homingCurrentSpikeCount++;
        } else {
            homingCurrentSpikeCount = 0; // Reset if current drops
        }
        
        // Check if we've detected a sustained current spike
        boolean homingComplete = homingCurrentSpikeCount >= TurretConstants.kHomingCurrentSpikeCountRequired;
        
        if (homingComplete) {
            // Stop the motor and reset encoder
            io.stop();
            io.resetEncoder(TurretConstants.kHomingHardstopPositionRad);
            homed = true;
            controlMode = ControlMode.DISABLED;
            homingCurrentSpikeCount = 0;
            
            // Set current position as commanded position
            commandedFieldAngleRad = getCurrentFieldAngleRad();
            commandedTurretAngleRad = getCurrentTurretAngleRad();
            
            Logger.recordOutput("Turret/HomingComplete", true);
        } else {
            // Continue moving slowly toward the hardstop
            io.setVoltage(TurretConstants.kHomingVoltage);
        }
        
        Logger.recordOutput("Turret/HomingCurrentSpikeCount", homingCurrentSpikeCount);
        Logger.recordOutput("Turret/CurrentSpikeDetected", currentSpikeDetected);
    }

    private void stopOutputs() {
        io.stop();
    }

    private double calculateRobotRelativeSetpoint(double fieldAngleRad) {
        double normalizedField = wrapAngle(fieldAngleRad);
        double robotHeading = wrapAngle(robotPoseSupplier.get().getRotation().getZ());
        
        
        // To find the necessary turret angle:
        // FieldTarget = RobotHeading + TurretOffset + TurretAngle
        // TurretAngle = FieldTarget - RobotHeading - TurretOffset
        double rawRel = normalizedField - robotHeading - TurretConstants.kTurretZeroOffsetRad;
        double robotRelative = wrapAngle(rawRel);
        
        // Handle wrapping into turret's range [-300°, 0°]
        // If angle is positive (0° to 180°), wrap it to negative equivalent
        if (robotRelative > 0) {
            robotRelative -= 2 * Math.PI; 
        }
        
        double clamped = MathUtil.clamp(robotRelative, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);

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
