package frc.robot.subsystems.Turret;

import static frc.lib.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

/**
 * Real-hardware implementation of {@link TurretIO} backed by a REV CANSparkMax
 * brushless controller.
 */
public class TurretIOSpark implements TurretIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput homeSwitch;
    private final SparkMaxConfig config;

    public TurretIOSpark() {
        motor = new SparkMax(TurretConstants.kTurretMotorId, MotorType.kBrushless);

        config = new SparkMaxConfig();
        config
            .idleMode(TurretConstants.kBrakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast)
            .inverted(TurretConstants.kMotorInverted)
            .smartCurrentLimit(TurretConstants.kCurrentLimitAmps)
            .voltageCompensation(12.0);

        config.encoder
            .positionConversionFactor(TurretConstants.kPositionConversionFactor)
            .velocityConversionFactor(TurretConstants.kVelocityConversionFactor);

        config.softLimit
            .forwardSoftLimit(TurretConstants.kMaxAngleRad)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.kMinAngleRad)
            .reverseSoftLimitEnabled(true);

        config.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
                .pid(
                        TurretConstants.kPositionKp,
                        TurretConstants.kPositionKi,
                        TurretConstants.kPositionKd)
                .outputRange(-1.0, 1.0);

        config.closedLoop.apply(closedLoopConfig);

        tryUntilOk(
                motor,
                5,
                () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        encoder = motor.getEncoder();
        encoder.setPosition(TurretConstants.kHomingSwitchZeroPositionRad);

        closedLoopController = motor.getClosedLoopController();

        if (TurretConstants.kUseHomingSwitch) {
            homeSwitch = new DigitalInput(TurretConstants.kHomingSwitchDioPort);
        } else {
            homeSwitch = null;
        }
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        ifOk(motor, encoder::getPosition, value -> inputs.positionRad = value);
        ifOk(motor, encoder::getVelocity, value -> inputs.velocityRadPerSec = value);
        ifOk(
                motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                values -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);
        ifOk(motor, motor::getMotorTemperature, value -> inputs.motorTempCelsius = value);

        inputs.zeroSwitchPressed = isHomeSwitchPressed();
        inputs.forwardLimit = inputs.positionRad >= TurretConstants.kMaxAngleRad;
        inputs.reverseLimit = inputs.positionRad <= TurretConstants.kMinAngleRad;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(MathUtil.clamp(volts, -TurretConstants.kMaxVoltage, TurretConstants.kMaxVoltage));
    }

    @Override
    public void setPositionSetpoint(double positionRad) {
        double clampedPosition = MathUtil.clamp(positionRad, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);

        REVLibError status = closedLoopController.setSetpoint(clampedPosition, ControlType.kPosition);

        if (status != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to set turret closed-loop setpoint: " + status.toString(),
                    false);
        }
    }

    @Override
    public void resetEncoder(double positionRad) {
        encoder.setPosition(positionRad);
    }

    // Overload that provides a default value of 0.0
    public void resetEncoder() {
        resetEncoder(0.0);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        SparkMaxConfig idleConfig = new SparkMaxConfig();
        idleConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                        idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configureClosedLoop(double kp, double ki, double kd){
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig(); // Only closed loop settings
        closedLoopConfig
                .pid(kp, ki, kd)
                .outputRange(-1.0, 1.0);

        config.closedLoop.apply(closedLoopConfig);

        tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    private boolean isHomeSwitchPressed() {
        if (homeSwitch == null) {
            return false;
        }

        boolean rawState = homeSwitch.get();
        return TurretConstants.kHomingSwitchNormallyOpen ? !rawState : rawState;
    }
}
