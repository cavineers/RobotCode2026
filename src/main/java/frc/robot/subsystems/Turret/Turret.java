package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private enum ControlMode {
        DISABLED,
        MANUAL,
        POSITION
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final DoubleSupplier robotHeadingSupplier;

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

    private boolean lastHomeSwitchState = false;

    public Turret(TurretIO io, DoubleSupplier robotHeadingSupplier) {
        this.io = io;
        this.robotHeadingSupplier = robotHeadingSupplier;
    }

    public Turret(TurretIO io) {
        this(io, () -> 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

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
        Logger.recordOutput(
        "Turret/FieldPose3d",
        new Pose3d(new Translation3d(), new Rotation3d(0.0, 0.0, getCurrentFieldAngleRad())));
    }

    public void setFieldRelativeTarget(double fieldAngleRad) {
        commandedFieldAngleRad = wrapAngle(fieldAngleRad);
        controlMode = ControlMode.POSITION;
    }

    public void setRobotRelativeTarget(double turretAngleRad) {
        double robotHeading = wrapAngle(robotHeadingSupplier.getAsDouble());
        double normalizedTurret = wrapAngle(turretAngleRad);
        setFieldRelativeTarget(normalizedTurret + robotHeading);
    }

    public void holdCurrentPosition() {
        commandedFieldAngleRad = getCurrentFieldAngleRad();
        controlMode = ControlMode.POSITION;
    }

    public void setManualVoltage(double volts) {
        manualDemandVolts = MathUtil.clamp(volts, -TurretConstants.kMaxVoltage, TurretConstants.kMaxVoltage);
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
        return wrapAngle(inputs.positionRad + robotHeadingSupplier.getAsDouble());
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

        if (pressed && !lastHomeSwitchState) {
            io.resetEncoder(TurretConstants.kHomingSwitchZeroPositionRad);
            homed = true;
            commandedFieldAngleRad = getCurrentFieldAngleRad();
            commandedTurretAngleRad = wrapAngle(0.0);
        }

        lastHomeSwitchState = pressed;
    }

    private double calculateRobotRelativeSetpoint(double fieldAngleRad) {
        double normalizedField = wrapAngle(fieldAngleRad);
        double robotHeading = wrapAngle(robotHeadingSupplier.getAsDouble());
        double robotRelative = wrapAngle(normalizedField - robotHeading);
        return MathUtil.clamp(robotRelative, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
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
        double wrapped = MathUtil.angleModulus(angleRad);
        if (wrapped < TurretConstants.kMinAngleRad) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

}
