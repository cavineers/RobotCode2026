package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase{
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public Command goToPresetCommand(double rotations) {
         return Commands.run(() -> {
            this.io.setClosedLoop(true);
            io.updateTurretPosition(rotations);
        }, this);
       
    }

    public boolean getTurretLimitSwitch(){
        return inputs.turretLimitSwitchPressed;
    }

    public Command setTurretVoltsCommand(double volts){
        return Commands.run(() -> io.setTurretVolts(volts), this).finallyDo(interrupted -> io.setTurretVolts(0));
    }

    public Command resetTurretPositionCommand(){
        return Commands.run(() -> io.resetTurretPosition(), this).finallyDo(interrupted -> io.setTurretVolts(0));
    }

    public Command rotateCommand(){
        return Commands.run(() -> io.rotate(), this).finallyDo(interrupted -> io.setTurretVolts(0.0));
    }

    public void setTurretVolts(double volts){
        io.setTurretVolts(volts);
    }
    
}
