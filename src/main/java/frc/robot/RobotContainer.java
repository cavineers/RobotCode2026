package frc.robot;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kTopVoltage;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveCommand;

import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.ModuleIOSpark;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

import frc.robot.subsystems.InBumperIntake.InBumperIntake;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIO;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIOSim;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIOSpark;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;

public class RobotContainer {

    // Subsystems
    public final SwerveDriveSubsystem drivetrain;
    public final InBumperIntake inBumperIntake;

    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroPigeonIO(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSpark());
                break;
            case SIM:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSim());
                break;
            default:
                // Replay
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {}
                    );
                inBumperIntake = new InBumperIntake(new InBumperIntakeIO() {});
                break;
        }
       
        configureButtonBindings();
        configureNamedCommands();

        // // // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // // Set up SysId routines
        // autoChooser.addOption(
        // "Drive Wheel Radius Characterization",
        // SystemIdCommands.wheelRadiusCharacterization(drivetrain));
        // autoChooser.addOption(
        // "Drive Simple FF Characterization",
        // SystemIdCommands.feedforwardCharacterization(drivetrain));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Forward)",
        // drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Reverse)",
        // drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Forward)",
        // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Reverse)",
        // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureButtonBindings() {
        // Set the drivetrain default command
        drivetrain.setDefaultCommand(new SwerveCommand(
                drivetrain,
                primaryDriverController::getLeftY,
                primaryDriverController::getLeftX,
                primaryDriverController::getRightX)
            );
        
        primaryDriverController.rightBumper().onTrue(inBumperIntake.setTopVoltageCommand(12*kTopVoltage));
        primaryDriverController.rightBumper().onFalse(inBumperIntake.setTopVoltageCommand(0));
        primaryDriverController.rightBumper().onTrue(inBumperIntake.setBottomVoltageCommand(-12*kBottomVoltage));
        primaryDriverController.rightBumper().onFalse(inBumperIntake.setBottomVoltageCommand(0));
        primaryDriverController.rightBumper().onTrue(inBumperIntake.setHopperVoltageCommand(12*kHopperVoltage));
        primaryDriverController.rightBumper().onFalse(inBumperIntake.setHopperVoltageCommand(0));

        primaryDriverController.leftBumper().onTrue(inBumperIntake.setTopVoltageCommand(-12*kTopVoltage));
        primaryDriverController.leftBumper().onFalse(inBumperIntake.setTopVoltageCommand(0));
        primaryDriverController.leftBumper().onTrue(inBumperIntake.setBottomVoltageCommand(-12*kBottomVoltage));
        primaryDriverController.leftBumper().onFalse(inBumperIntake.setBottomVoltageCommand(0));
        primaryDriverController.leftBumper().onTrue(inBumperIntake.setHopperVoltageCommand(12*kHopperVoltage));
        primaryDriverController.leftBumper().onFalse(inBumperIntake.setHopperVoltageCommand(0));

        primaryDriverController.rightTrigger().onTrue(inBumperIntake.setTopVoltageCommand(12*kTopVoltage));
        primaryDriverController.rightTrigger().onFalse(inBumperIntake.setTopVoltageCommand(0));
        primaryDriverController.rightTrigger().onTrue(inBumperIntake.setBottomVoltageCommand(-12*kBottomVoltage));
        primaryDriverController.rightTrigger().onFalse(inBumperIntake.setBottomVoltageCommand(0));
        primaryDriverController.rightTrigger().onTrue(inBumperIntake.setHopperVoltageCommand(-12*kHopperVoltage));
        primaryDriverController.rightTrigger().onFalse(inBumperIntake.setHopperVoltageCommand(0));
    }

    public void configureNamedCommands() {
        // Register Named Commands
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}