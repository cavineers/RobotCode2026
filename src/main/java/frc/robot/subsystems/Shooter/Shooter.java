package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io){
        this.io = io;
    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs); 
        Logger.processInputs("Shooter", inputs);
    }
    
    public boolean getIR() {
        return inputs.ShooterIR;
    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
    }

    public void setPercentage(double percentage){
        io.setVoltage(percentage * 12);
    }

    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    public void setFF(double kS, double kV, double kA) {
        io.setFF(kS, kV, kA);
    }
    
    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this);
    }

    public Command stopCommand() {
        return Commands.run(() -> io.setVoltage(0), this);
    }
}
