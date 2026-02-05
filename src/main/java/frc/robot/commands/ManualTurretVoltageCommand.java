package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;

/**
 * Default command for the turret that continuously applies manual voltage based on the
 * secondary driver's right stick axis.
 */
public class ManualTurretVoltageCommand extends Command {
    private final Turret turret;
    private final DoubleSupplier axisSupplier;

    private static final double AXIS_DEADBAND = 0.05;

    public ManualTurretVoltageCommand(Turret turret, DoubleSupplier axisSupplier) {
        this.turret = turret;
        this.axisSupplier = axisSupplier;
        addRequirements(turret);
    }


    @Override
    public void initialize() {
        turret.enableClosedLoop(false);
    }

    @Override
    public void execute() {
        double rawAxis = axisSupplier.getAsDouble();
        double processedAxis = MathUtil.applyDeadband(rawAxis, AXIS_DEADBAND);
        turret.setManualVoltage(processedAxis * TurretConstants.kMaxVoltage / 2.0);
        Logger.recordOutput("Turret/ManualAxis", rawAxis);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
