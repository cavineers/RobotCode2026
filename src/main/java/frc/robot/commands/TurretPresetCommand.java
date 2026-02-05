package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import org.littletonrobotics.junction.Logger;

/**
 * Command that drives the turret to a predefined preset angle using closed-loop control.
 * Designed to be bound to controller buttons (e.g. via {@code whileTrue} bindings).
 */
public class TurretPresetCommand extends Command {
    private final Turret turret;
    private final double presetAngleRad;
    private final String presetName;

    public TurretPresetCommand(Turret turret, double presetAngleRad, String presetName) {
        this.turret = turret;
        this.presetAngleRad = presetAngleRad;
        this.presetName = presetName;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enableClosedLoop(true);
        Logger.recordOutput("Turret/ActivePreset", presetName);
    }

    @Override
    public void execute() {
        turret.setRobotRelativeTarget(presetAngleRad);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Turret/ActivePreset", "Idle");
        if (!interrupted) {
            turret.holdCurrentPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
