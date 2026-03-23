package frc.robot;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kOutsideVoltage;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOSpark;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ShooterCharacterizationCommand;
import frc.robot.commands.SwerveCommand;
import frc.lib.FuelSim;

import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.OverBumperIntake.OverBumperIntake;
import frc.robot.subsystems.OverBumperIntake.OverBumperIntakeIO;
import frc.robot.subsystems.OverBumperIntake.OverBumperIntakeIOSim;
import frc.robot.subsystems.OverBumperIntake.OverBumperIntakeIOSpark;
import frc.robot.subsystems.Drivetrain.ModuleIOTalonFX;
import frc.robot.commands.SystemIdCommands;
import frc.robot.subsystems.InBumperIntake.InBumperIntake;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIO;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIOSim;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIOSpark;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOKraken;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

    // Subsystems
    public final Turret turret;
    public final SwerveDriveSubsystem drivetrain;
    public final OverBumperIntake overBumperIntake;
    public final InBumperIntake inBumperIntake;
    public final ShooterSubsystem shooter;
    public final Vision vision;

    /** Fuel particle simulation — only non-null in SIM mode. */
    public FuelSim fuelSim = null;

    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandGenericHID secondaryDriverController = new CommandGenericHID(1);

    // Manual override state
    private double shooterRPMOverride = 3000.0; // Starting RPM for override mode

    // Auto chooser
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroPigeonIO(),
                        new ModuleIOTalonFX(0),
                        new ModuleIOTalonFX(1),
                        new ModuleIOTalonFX(2),
                        new ModuleIOTalonFX(3));
                overBumperIntake = new OverBumperIntake(new OverBumperIntakeIOSpark());
                turret = new Turret(new TurretIOSpark(), () -> new Pose3d(drivetrain.getPose()));
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSpark());
                shooter = new ShooterSubsystem(
                        new ShooterIOKraken());
                vision = new Vision(
                    drivetrain::addVisionMeasurement,
                    drivetrain::resetOdometry,
                    new VisionIOPhotonVision(VisionConstants.camera1Name, (timestamp) -> VisionConstants.robotToCamera1),
                    new VisionIOPhotonVision(VisionConstants.camera2Name, (timestamp) -> VisionConstants.robotToCamera2));
                //     new VisionIOPhotonVision(VisionConstants.camera2Name, (timestamp) -> {
                //         // Calculate turret-adjusted transform for moving camera using historical position
                //         Rotation2d turretAngle = turret.getTurretAngleAtTime(timestamp);
                        
                //         // Create a rotation-only transform at the turret center
                //         Transform3d turretRotation = new Transform3d(
                //             new Translation3d(), // No translation, just rotation
                //             new Rotation3d(0, 0, turretAngle.getRadians())
                //         );
                        
                //         // Chain transforms: Robot->TurretCenter, then rotate, then Turret->Camera
                //         // This ensures turretToCamera2 is applied in the rotated turret's coordinate frame
                //         return VisionConstants.robotToTurretCenter
                //             .plus(turretRotation)
                //             .plus(VisionConstants.turretToCamera2);
                //     })
                
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
                overBumperIntake = new OverBumperIntake(new OverBumperIntakeIOSim());
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSim());
                shooter = new ShooterSubsystem(
                        new ShooterIOSim());
                
                vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::resetOdometry, new VisionIO(){});

                // ---- FuelSim ----
                fuelSim = new FuelSim("FuelSim/Fuel");
                fuelSim.registerRobot(
                        Constants.RobotDimensions.kWidthMeters,
                        Constants.RobotDimensions.kLengthMeters,
                        Constants.RobotDimensions.kBumperHeightMeters,
                        drivetrain::getPose,
                        drivetrain::getFieldRelativeChassisSpeeds);
                fuelSim.setLoggingFrequency(100);
                fuelSim.setSubticks(10);
                fuelSim.start();
                break;
            default:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                overBumperIntake = new OverBumperIntake(new OverBumperIntakeIO() {
                });

                turret = new Turret(new TurretIO() {
                }, () -> new Pose3d());
                inBumperIntake = new InBumperIntake(new InBumperIntakeIO() {
                });
                shooter = new ShooterSubsystem(
                        new ShooterIO() {
                        });

                vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::resetOdometry, new VisionIO(){});
                break;
        }

        configureButtonBindings();
        configureNamedCommands();
        configureAutoChooser();
        configureElasticWidgets();

    }

    private void configureButtonBindings() {
        // Set default drivetrain command — halve speed while auto-shooting (left trigger)
        drivetrain.setDefaultCommand(new SwerveCommand(
                drivetrain,
                drivetrain::getAlignTrench,
                primaryDriverController::getLeftY,
                primaryDriverController::getLeftX,
                primaryDriverController::getRightX,
                () -> primaryDriverController.getRightTriggerAxis() > 0.5 ? 0.5 : 1.0)
            );
               
        // ------ PRIMARY DRIVER CONTROLS ------
        
        primaryDriverController.b().onTrue(overBumperIntake.deployCommand());
                
        // Toggle InBumperIntake: Press to start runGroundToHopper, press again to stop
        primaryDriverController.x().toggleOnTrue(
            inBumperIntake.runGroundToHopper(
                InBumperIntakeConstants.kOutsideVoltage, 
                InBumperIntakeConstants.kBottomVoltage, 
                InBumperIntakeConstants.kTopVoltage)
        );

        // TODO: PLACEHOLDER: replace with actual button
        var manualOverrideSwitch = secondaryDriverController.button(8);

        // Right trigger: Auto shoot OR hopper to shooter (depending on manual override)
        primaryDriverController.rightTrigger().toggleOnTrue(
            Commands.deferredProxy(() ->
                manualOverrideSwitch.getAsBoolean() ?
                    // Manual override mode: toggle hopper to shooter
                    inBumperIntake.runHopperToShooter(
                        InBumperIntakeConstants.kOutsideVoltage,
                        InBumperIntakeConstants.kBottomVoltage,
                        InBumperIntakeConstants.kTopVoltage)
                    :
                    // Normal mode: auto shoot
                    new AutoShootCommand(drivetrain, shooter, turret, inBumperIntake, fuelSim)
            )
        );

        // ------ SECONDARY DRIVER CONTROLS ------
        
        // Button 2: Start turret homing sequence (current-based hardstop detection)
        secondaryDriverController.button(11).onTrue(
            Commands.runOnce(() -> turret.startHoming(), turret)
        );

        // Turret left/right: axis acts as a button, so bind whileTrue on each direction
        secondaryDriverController.axisGreaterThan(0, 0.5).whileTrue(
            Commands.run(() -> turret.setRobotRelativeTarget(
                turret.getTargetTurretAngleRad() + Math.toRadians(2.0)), turret)
        );
        secondaryDriverController.axisLessThan(0, -0.5).whileTrue(
            Commands.run(() -> turret.setRobotRelativeTarget(
                turret.getTargetTurretAngleRad() - Math.toRadians(2.0)), turret)
        );

        // ------ IN-BUMPER INTAKE MODES (secondary buttons 3/4/5) ------
        // Button 3: Toggle ground -> hopper
        secondaryDriverController.button(3).toggleOnTrue(
            inBumperIntake.runGroundToHopper(
                InBumperIntakeConstants.kOutsideVoltage,
                InBumperIntakeConstants.kBottomVoltage,
                InBumperIntakeConstants.kTopVoltage)
        );

        // Button 4: Toggle ground -> shooter (bypass hopper)
        secondaryDriverController.button(2).toggleOnTrue(
            inBumperIntake.runGroundToShooter(
                InBumperIntakeConstants.kOutsideVoltage,
                InBumperIntakeConstants.kBottomVoltage,
                InBumperIntakeConstants.kTopVoltage)
        );

        // Button 5: Toggle hopper -> shooter
        secondaryDriverController.button(1).toggleOnTrue(
            inBumperIntake.runHopperToShooter(
                InBumperIntakeConstants.kOutsideVoltage,
                InBumperIntakeConstants.kBottomVoltage,
                InBumperIntakeConstants.kTopVoltage)
        );

        // OverBumper Unjam Sequence
        secondaryDriverController.button(12).toggleOnTrue(
            Commands.deferredProxy(() -> overBumperIntake.unjamCommand())
        );
    }

    public void configureAutoChooser() {
        // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());// 
        // Set up SysId routines
        autoChooser.addOption("Shooter Characterization",
                ShooterCharacterizationCommand.feedforwardCharacterization(shooter));

        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                SystemIdCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption(
                "Drive Base Radius Characterization",
                SystemIdCommands.driveBaseRadiusCharacterization(drivetrain));
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

        // Rotation SysId for trackwidth characterization
        autoChooser.addOption(
                "Rotation SysId (Quasistatic Forward)",
                drivetrain.sysIdRotationQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Rotation SysId (Quasistatic Reverse)",
                drivetrain.sysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Rotation SysId (Dynamic Forward)",
                drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Rotation SysId (Dynamic Reverse)",
                drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void configureElasticWidgets(){
        // Register Elastic widgets (see frc.lib.Elastic)
    }

    public void configureNamedCommands() {
        // Register Named Commands
        NamedCommands.registerCommand("unjam_otb", overBumperIntake.unjamCommand());
        NamedCommands.registerCommand("intake_otb", overBumperIntake.deployCommand());
        NamedCommands.registerCommand("stop_otb", overBumperIntake.stopCommand());
        NamedCommands.registerCommand("auto_shoot", new AutoShootCommand(drivetrain, shooter, turret, inBumperIntake));
        NamedCommands.registerCommand("stop_shooter", 
            Commands.parallel(
                shooter.stopCommand(),
                inBumperIntake.stopCommand()
            )
        );
        NamedCommands.registerCommand("home_turret", Commands.runOnce(() -> turret.startHoming(), turret));
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }
}