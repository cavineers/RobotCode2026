package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.commands.ManualTurretVoltageCommand;
import frc.robot.commands.TurretPresetCommand;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOSpark;
import frc.robot.subsystems.Turret.TurretConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveCommand;

import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.ModuleIOSpark;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOKraken;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.commands.SystemIdCommands;

public class RobotContainer {

    // Subsystems
    private final Turret turret;
    // private final SparkMax testGyro; 
    // private final RelativeEncoder testGyroEncoder;
    public final SwerveDriveSubsystem drivetrain;
    // public final ShooterSubsystem shooter;

    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    // Auto chooser
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        // testGyro = new SparkMax(50, MotorType.kBrushless);
        // testGyroEncoder = testGyro.getEncoder();

        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroPigeonIO(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));

                turret = new Turret(new TurretIOSpark(), () -> new Pose3d(drivetrain.getPose()));
                break;
            case SIM:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                turret = new Turret(new TurretIOSim(), () -> new Pose3d(drivetrain.getPose()));
                break;
            default:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                turret = new Turret(new TurretIO() {
                }, () -> new Pose3d());
                break;
        }

        turret.setDefaultCommand(
                new ManualTurretVoltageCommand(turret, () -> secondaryDriverController.getHID().getRawAxis(0)));

        configureButtonBindings();
        configureNamedCommands();

        
    }

    private void configureButtonBindings() {
        secondaryDriverController.a().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetOneRad, "One"));
        secondaryDriverController.b().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetTwoRad, "Two"));
        secondaryDriverController.y().whileTrue(
                new TurretPresetCommand(turret, TurretConstants.kPresetThreeRad, "Three"));
        
        
        // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        SystemIdCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption(
        "Drive Simple FF Characterization",
        SystemIdCommands.feedforwardCharacterization(drivetrain));
        autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        drivetrain.setDefaultCommand(new SwerveCommand(
            drivetrain,
            primaryDriverController::getLeftY,
            primaryDriverController::getLeftX,
            primaryDriverController::getRightX)
        );
    }
    
    public void configureNamedCommands() {
        // Register Named Commands
    }

    public Command getAutonomousCommand() {
        return null;
    }
}