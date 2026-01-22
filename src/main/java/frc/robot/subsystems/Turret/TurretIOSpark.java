package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.kTurretCanID;
import static frc.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;



public class TurretIOSpark implements TurretIO{
    private final SparkFlex motor = new SparkFlex(kTurretCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private final DigitalInput turretLimitSwitch = new DigitalInput(TurretConstants.kTurretLimitSwitchID);

    private final SparkFlexConfig config;

    
    
    public TurretIOSpark() {
        config = new SparkFlexConfig();
        config
             .inverted(TurretConstants.kInverted)
             .idleMode(TurretConstants.kIdleMode)
             .smartCurrentLimit((int) TurretConstants.kCurrentLimit)
             .voltageCompensation(12.0);
        config.signals
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        tryUntilOk(
            motor,
            5, 
            ()  -> motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));;

    
    }
 @Override
    public void updateInputs(TurretIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.turretPositionRad = value); 
        ifOk(motor, encoder::getVelocity, (value) -> inputs.turretVelocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.turretAppliedVoltage = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.turretCurrentAmps = value);
    }

    

    public boolean getSensor(DigitalInput sensor){
        return turretLimitSwitch.get();
    }

    @Override
    public void setTurretVolts(double volts) {
        motor.setVoltage(volts);
    }
    @Override
    public void setTurretPosition(double positionRad) {
      ;
    }
    @Override
    public void resetTurretPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetTurretPosition'");
    } 
}



 



