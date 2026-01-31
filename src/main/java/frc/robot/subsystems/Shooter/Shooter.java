package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double kP;
    private double kI;
    private double kD;

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Shooter/kP", ShooterConstants.kP);
    private LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Shooter/kI", ShooterConstants.kI);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Shooter/kD", ShooterConstants.kD);

    public Shooter(ShooterIO io){
        this.io = io;
    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs);

        if (kP != tuningP.get() || kI != tuningI.get() || kD != tuningD.get()) {
            kP = tuningP.get();
            kI = tuningI.get();
            kD = tuningD.get();
            setPID(kP, kI, kD);
        }

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
    public void setVelocity(double velocityRPM) {
        Logger.recordOutput("Shooter/VelocitySetpoint", velocityRPM, Units.RPM);
        io.setVelocity(velocityRPM);
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
