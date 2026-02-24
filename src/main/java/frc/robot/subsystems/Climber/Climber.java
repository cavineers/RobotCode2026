package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.subsystems.Climber.ClimberConstants.kDeployedMotorRotations;
import static frc.robot.subsystems.Climber.ClimberConstants.kRestMotorRotations;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double kP;
    private double kI;
    private double kD;

    public enum ClimbState{
        RESTING,
        DEPLOYED,
        ENGAGED
    }

    HashMap<Double, ClimbState> climbStater = new HashMap<>();
    
    @AutoLogOutput(key="Climber/ClimbState")
    public ClimbState climbState = ClimbState.RESTING;

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Climber/kP", ClimberConstants.kP);
    private LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Climber/kI", ClimberConstants.kI);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Climber/kD", ClimberConstants.kD);
    
    public Climber(ClimberIO io) {
        this.io = io;
        climbStater.put(ClimberConstants.kRestMotorRotations, ClimbState.RESTING);
        climbStater.put(ClimberConstants.kDeployedMotorRotations, ClimbState.DEPLOYED);
        climbStater.put(ClimberConstants.kEngagedMotorRotations, ClimbState.ENGAGED);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (kP != tuningP.get() || kI != tuningI.get() || kD != tuningD.get()) {
            kP = tuningP.get();
            kI = tuningI.get();
            kD = tuningD.get();
            this.io.setPID(kP, kI, kD);
        }

        climbState = climbStater.get(getSetpoint());
        Logger.processInputs("Climber", inputs);

        if (inputs.cutoff && climbState == ClimbState.RESTING) {
            io.resetEncoder(ClimberConstants.kRestMotorRotations);
        }
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> {
            io.setClimberVoltage(volts);
        }, this);
    } 
    public Command goToPresetCommand() {
        if (climbState == ClimbState.RESTING) {
            return Commands.runOnce(() -> {
            io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
            }, this);
        }
        else if (climbState == ClimbState.DEPLOYED) {
            return Commands.runOnce(() -> {
            io.updateClimberSetpoint(ClimberConstants.kEngagedMotorRotations);
            }, this);
        }
        else {
            return Commands.runOnce(() -> {
            io.updateClimberSetpoint(ClimberConstants.kRestMotorRotations);
            }, this);
        }
    }

    public Command releaseAutoCommand() {
        return Commands.runOnce(() -> {
        io.updateClimberSetpoint(ClimberConstants.kRestMotorRotations);
        }, this);
    }

    public double getClimberPosition() {
        return inputs.climberPositionRotations;
    }

	public double getSetpoint() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getSetpoint'");
	}
}