package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
    private final SparkMax testGyro; 
    private final RelativeEncoder testGyroEncoder;
    // Controllers
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    // Auto chooser
    // private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        testGyro = new SparkMax(50, MotorType.kBrushless);
        testGyroEncoder = testGyro.getEncoder();

        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                turret = new Turret(new TurretIOSpark(), () -> testGyroEncoder.getPosition() * 2 * Math.PI);
                break;
            case SIM:
                turret = new Turret(new TurretIOSim(), () -> 0.0);
                break;
            default:
                turret = new Turret(new TurretIO() {
                }, () -> 0.0);
                break;
        }

        // turret.setDefaultCommand(
        //         new ManualTurretVoltageCommand(turret, () -> secondaryDriverController.getHID().getRawAxis(0)));

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