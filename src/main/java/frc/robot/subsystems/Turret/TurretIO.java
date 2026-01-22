package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

   

    @AutoLog
    public class TurretIOInputs {
        public double turretPositionRad = 0.0;
        public double turretVelocityRadPerSec = 0.0;
        public double turretCurrentAmps = 0.0;
        public double turretAppliedVoltage = 0.0;

        public boolean turretLimitSwitchPressed = false;  
    }

    public void updateInputs(TurretIOInputs inputs);

    public void setTurretPosition(double positionRad);

    public void resetTurretPosition();

    public void setTurretVolts(double volts);

} 