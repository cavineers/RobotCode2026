package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import static frc.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;


public class ShooterIOSpark implements ShooterIO {
    private final SparkMax motor = new SparkMax(ShooterConstants.kFlywheelCanID, MotorType.kBrushless);

    private final RelativeEncoder encoder = motor.getEncoder();

    private final DigitalInput ShooterIR = new DigitalInput(ShooterConstants.kShooterIR);

    private SparkMaxConfig config;


    public ShooterIOSpark() {
        config = new SparkMaxConfig();
        config
            .inverted(ShooterConstants.kInverted)
            .idleMode(ShooterConstants.kIdleMode)
            .smartCurrentLimit(ShooterConstants.kCurrentLimit)
            .voltageCompensation(12);
        config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.hoodPositionRad = value); 
        ifOk(motor, encoder::getVelocity, (value) -> inputs.hoodVelocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    
        inputs.ShooterIR = this.ShooterIR.get();
    }

    public boolean getIR(){
        return this.ShooterIR.get();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}