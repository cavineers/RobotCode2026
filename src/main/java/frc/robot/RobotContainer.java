package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ManualTurretVoltageCommand;
import frc.robot.commands.TurretPresetCommand;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOSpark;
import frc.robot.subsystems.Turret.TurretConstants;

public class RobotContainer {

    // Subsystems
    private final Turret turret;
    // Controllers
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    // Auto chooser
    // private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                turret = new Turret(new TurretIOSpark(), () -> 0.0);
                break;
            case SIM:
                turret = new Turret(new TurretIOSim(), () -> 0.0);
                break;
            default:
                turret = new Turret(new TurretIO() {
                }, () -> 0.0);
                break;
        }

        turret.setDefaultCommand(
                new ManualTurretVoltageCommand(turret, () -> secondaryDriverController.getHID().getRawAxis(0)));

        configureButtonBindings();
        configureNamedCommands();

        // Set up auto routines for SysIds
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

    }

    private void configureButtonBindings() {
        secondaryDriverController.a().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetOneRad, "One"));
        secondaryDriverController.b().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetTwoRad, "Two"));
        secondaryDriverController.y().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetThreeRad, "Three"));
    }

    public void configureNamedCommands() {
        // Register Named Commands
    }

    public Command getAutonomousCommand() {
        return null;
    }
}