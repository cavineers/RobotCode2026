package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.kTurretLimitSwitchID;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
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


    public boolean getTurretLimitSwitch(DigitalInput turretLimitSwitch){
        return inputs.turretLimitSwitchPressed;
    }
    
    

    
}
