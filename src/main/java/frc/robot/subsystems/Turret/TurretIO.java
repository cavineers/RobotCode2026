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

    default void updateInputs(TurretIOInputs inputs){
    }

    public default void updateTurretPosition(double Pose2d){
    }

    public default void setClosedLoop(boolean isClosed){}


    public default void resetTurretPosition(){
    }

    public default void setTurretVoltage(double volts){
    }

    public default void rotate(){
    }

} 