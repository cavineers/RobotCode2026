package frc.robot;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kOutsideVoltage;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.commands.ManualTurretVoltageCommand;
import frc.robot.commands.TurretPresetCommand;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOSpark;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Turret.TurretConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ShooterCharacterizationCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.ShooterTableCharacterizationCommand;
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

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;

import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOKraken;
import frc.robot.subsystems.Climber.ClimberIOSim;

public class RobotContainer {

    // Subsystems

    private final Turret turret;
    public final SwerveDriveSubsystem drivetrain;
    public final Climber climber;
    public final OverBumperIntake overBumperIntake;
    public final InBumperIntake inBumperIntake;
    public final ShooterSubsystem shooter;
    public final Vision vision;

    /** Fuel particle simulation — only non-null in SIM mode. */
    public FuelSim fuelSim = null;

    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

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
                climber = new Climber(
                        new ClimberIOKraken());
                overBumperIntake = new OverBumperIntake(new OverBumperIntakeIOSpark());
                turret = new Turret(new TurretIOSpark(), () -> new Pose3d(drivetrain.getPose()));
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSpark());
                shooter = new ShooterSubsystem(
                        new ShooterIOKraken());
                vision = new Vision(drivetrain::addVisionMeasurement, new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
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
                climber = new Climber(new ClimberIOSim());
                overBumperIntake = new OverBumperIntake(new OverBumperIntakeIOSim());
                inBumperIntake = new InBumperIntake(new InBumperIntakeIOSim());
                shooter = new ShooterSubsystem(
                        new ShooterIOSim());
                
                vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO(){});

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
                climber = new Climber(
                        new ClimberIO() {
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

                vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO(){});
                break;
        }

        configureButtonBindings();
        configureNamedCommands();
        configureAutoChooser();

    }

    private void configureButtonBindings() {
        // Set default drivetrain command
        drivetrain.setDefaultCommand(new SwerveCommand(
                drivetrain,
                primaryDriverController::getLeftY,
                primaryDriverController::getLeftX,
                primaryDriverController::getRightX)
            );
        
        // ------ PRIMARY DRIVER CONTROLS ------
        // Shooting controls (for testing purposes, will be replaced with vision-based shooting commands)
        // primaryDriverController.rightTrigger().whileTrue(
        //     Commands.run(
        //         () -> shooter.setTunableVelocity(), shooter)
        //     .finallyDo(() -> shooter.stop()));
        
        primaryDriverController.b().onTrue(overBumperIntake.deployCommand());
        
        // Toggle InBumperIntake: Press to start runGroundToHopper, press again to stop
        primaryDriverController.x().toggleOnTrue(
            inBumperIntake.runGroundToHopper(
                InBumperIntakeConstants.kOutsideVoltage, 
                InBumperIntakeConstants.kBottomVoltage, 
                InBumperIntakeConstants.kTopVoltage)
        );
        primaryDriverController.rightTrigger().toggleOnTrue(
                inBumperIntake.runHopperToShooter(
                InBumperIntakeConstants.kOutsideVoltage, 
                InBumperIntakeConstants.kBottomVoltage, 
                InBumperIntakeConstants.kTopVoltage)
        );

        // Auto shoot
        primaryDriverController.a().whileTrue(new AutoShootCommand(drivetrain, shooter, turret, fuelSim));

        // ------ SECONDARY DRIVER CONTROLS ------
        // Y: Hold to spin up at tunable RPM + pitch and aim at goal for LUT characterization.
        //    While held, press A to fire a sim shot and log the data point.
        // ShooterTableCharacterizationCommand charCmd =
        //         new ShooterTableCharacterizationCommand(drivetrain, shooter, turret, fuelSim);
        // secondaryDriverController.y().whileTrue(charCmd);
        // secondaryDriverController.a().onTrue(
        //         edu.wpi.first.wpilibj2.command.Commands.runOnce(charCmd::fire));
        secondaryDriverController.y().whileTrue(Commands.run(()-> shooter.setTunableVelocity(), shooter).finallyDo(() -> shooter.setVelocity(0)));
        // POV Up: Hold turret at 0 degrees field-relative (due north)
        // secondaryDriverController.povUp().onTrue(
        //     Commands.runOnce(
        //         () -> turret.setFieldRelativeTarget(0.0),
        //         turret
        //     )
        // );

        // POV Right: Hold turret at 90 degrees field-relative (due east)
        // secondaryDriverController.povRight().onTrue(
        //     Commands.runOnce(
        //         () -> turret.setFieldRelativeTarget(-90.0),
        //         turret
        //     )
        // );

        // // POV Left: Hold turret at -90 degrees field-relative (due west)
        // secondaryDriverController.povLeft().onTrue(
        //     Commands.runOnce(
        //         () -> turret.setFieldRelativeTarget(Math.toRadians(90.0)),
        //         turret
        //     )
        // );


    }

    public void configureAutoChooser() {
        // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");// AutoBuilder.buildAutoChooser()
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

    public void configureNamedCommands() {
        // Register Named Commands
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    /**
     * @brief Releases the climber to the extended position
     * @Note Climber must have the setpoint set to kDeploy
     */
    public void releaseAutoClimb() {
        if (climber.getSetpoint() == ClimberConstants.kEngagedMotorRotations) {
            CommandScheduler.getInstance().schedule(climber.releaseAutoCommand());
        }
    }
}