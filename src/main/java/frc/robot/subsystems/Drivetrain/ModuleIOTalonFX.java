package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants.*;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.ModuleConstants.*;
import static frc.lib.PhoenixUtil.*;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(int module) {
    driveTalon = new TalonFX(switch (module) {
        case 0 -> kFrontLeftDriveCanID;
        case 1 -> kFrontRightDriveCanID;
        case 2 -> kBackLeftDriveCanID;
        case 3 -> kBackRightDriveCanID;
        default -> 0;
    }, kCANBus);
    turnTalon = new TalonFX(switch (module) {
        case 0 -> kFrontLeftTurningCanID;
        case 1 -> kFrontRightTurningCanID;
        case 2 -> kBackLeftTurningCanID;
        case 3 -> kBackRightTurningCanID;
        default -> 0;
    }, kCANBus);
    cancoder = new CANcoder(switch (module) {
        case 0 -> kFrontLeftAbsoluteEncoderPort;
        case 1 -> kFrontRightAbsoluteEncoderPort;
        case 2 -> kBackLeftAbsoluteEncoderPort;
        case 3 -> kBackRightAbsoluteEncoderPort;
        default -> 0;
    }, kCANBus);

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // driveConfig.Slot0 = constants.DriveMotorGains; // TODO: Needs Kp, Ki, Kd in constants
    driveConfig.Slot0.kP = kDriveKp;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = kDriveKd;
    driveConfig.Slot0.kS = kDriveKs;
    driveConfig.Slot0.kV = kDriveKv;
    
    // SensorToMechanismRatio set to 1.0 because Module.java handles gear ratio conversion
    driveConfig.Feedback.SensorToMechanismRatio = 1.0;
    
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = kDriveMotorCurrentLimit;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kDriveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimit = kDriveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    driveConfig.MotorOutput.Inverted =
        (switch (module) {
            case 0 -> kFrontLeftDriveEncoderReversed;
            case 1 -> kFrontRightDriveEncoderReversed;
            case 2 -> kBackLeftDriveEncoderReversed;
            case 3 -> kBackRightDriveEncoderReversed;
            default -> false;
        })
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // turnConfig.Slot0 = constants.SteerMotorGains; // TODO: Needs Kp, Ki, Kd in constants
    turnConfig.Slot0.kP = kTurnKp;
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = kTurnKd;

    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
     // FusedCANcoder is common for swerve
    turnConfig.Feedback.FeedbackSensorSource = switch (kTurnFeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new RuntimeException(
                  "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    
    turnConfig.Feedback.RotorToSensorRatio = kTurningMotorGearRatio;
    
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve
    
    turnConfig.MotorOutput.Inverted = 
        (switch (module) {
            case 0 -> kFrontLeftTurningEncoderReversed;
            case 1 -> kFrontRightTurningEncoderReversed;
            case 2 -> kBackLeftTurningEncoderReversed;
            case 3 -> kBackRightTurningEncoderReversed;
            default -> false;
        })
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    turnConfig.CurrentLimits.StatorCurrentLimit = kTurnMotorCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANcoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = 
        switch (module) {
            case 0 -> kFrontLeftAbsoluteEncoderOffset;
            case 1 -> kFrontRightAbsoluteEncoderOffset;
            case 2 -> kBackLeftAbsoluteEncoderOffset;
            case 3 -> kBackRightAbsoluteEncoderOffset;
            default -> 0.0;
        };
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));
    
    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());

    // Configure periodic status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(
        driveTalon, turnTalon, cancoder);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh signals
    var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());

    // Convert motor shaft rotations to wheel radians (divide by gear ratio since SensorToMechanismRatio = 1.0)
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / kDriveMotorGearRatio;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / kDriveMotorGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / kDriveMotorGearRatio)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (kDriveClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (kSteerClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (kDriveClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (kSteerClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }
}