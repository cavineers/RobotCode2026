package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;
import static frc.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;



public class TurretIOSpark implements TurretIO{
    private final SparkFlex motor = new SparkFlex(kTurretCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private PIDController controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);

    private final DigitalInput turretLimitSwitch = new DigitalInput(TurretConstants.kTurretLimitSwitchID);

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Turret/P", TurretConstants.kProportionalGainSpark);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Turret/D", TurretConstants.kDerivativeTermSpark);

    @AutoLogOutput
    private double absAngle;

    private final SparkFlexConfig config;

    public TurretIOSpark() {
        config = new SparkFlexConfig();
        config
             .inverted(TurretConstants.kInverted)
             .idleMode(TurretConstants.kIdleMode)
             .smartCurrentLimit((int) TurretConstants.kCurrentLimit)
             .voltageCompensation(12.0);
             // positionConversionFactor MotorRotations -> Turretrads
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

        if(TurretConstants.kTuningMode){
            this.updatePID();
        }
    }

    public boolean getSensor(DigitalInput sensor){
        return turretLimitSwitch.get();
    }

    @Override
    public void setTurretVoltage(double volts) {
        motor.setVoltage(volts);
    }
    @Override
    public void updateTurretPosition(double angle) {
      this.absAngle = this.clipAngle(angle);
    }

     public double clipAngle(double angle) {
        // if(absAngle < TurretConstants.kMaxAngle) {
        //     return TurretConstants.kMaxAngle;
        // } else if(absAngle < TurretConstants.kMinAngle) {
        //     return TurretConstants.kMinAngle;
        // }
        return angle;
    }

    private void updatePID() {
        double currentP = this.controller.getP();
        double currentD = this.controller.getD();

        if (currentP != this.tuningP.get() || currentD != this.tuningD.get()){
            this.controller.setPID(this.tuningP.get(), 0, this.tuningD.get());
        }
    }

    
    public void rotate(){
        updateTurretPosition(kMinAngleRad);
        setTurretVoltage(12.0);
    }

    public void resetTurretPosition(){
        updateTurretPosition(kMinAngleRad);
        setTurretVoltage(-12.0);
    }
}



 



