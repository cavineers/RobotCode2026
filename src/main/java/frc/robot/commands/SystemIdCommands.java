package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

public class SystemIdCommands {
    
    private SystemIdCommands() {} // private constructor to prevent instantiation


    public static Command feedforwardCharacterization(SwerveDriveSubsystem drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                        () -> {
                            drive.runCharacterization(0.0);
                        },
                        drive)
                        .withTimeout(2.0),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                // Ramp at 0.1 V/s (reaches 12V in 120 seconds)
                Commands.run(
                        () -> {
                            double voltage = timer.get() * 0.1;
                            drive.runCharacterization(voltage);
                            double velocity = drive.getCharacterizationAverageVelocity();
                            
                            // Only record samples after velocity stabilizes (> 50 rad/s motor speed)
                            // and voltage is reasonable (< 11V to leave headroom)
                            if (velocity > 1.0 && voltage < 11.0) {
                                velocitySamples.add(velocity);
                                voltageSamples.add(voltage);
                            }
                        },
                        drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    if (n < 10) {
                                        System.out.println("********** Drive FF Characterization FAILED **********");
                                        System.out.println("\tNot enough samples collected: " + n);
                                        System.out.println("\tRun the command for at least 8 seconds");
                                        return;
                                    }
                                    
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tSamples collected: " + n);
                                    System.out.println("\tVelocity range: " + 
                                        formatter.format(velocitySamples.stream().min(Double::compare).orElse(0.0)) + 
                                        " to " + 
                                        formatter.format(velocitySamples.stream().max(Double::compare).orElse(0.0)) + 
                                        " rad/s (MOTOR)");
                                    System.out.println("\tkS: " + formatter.format(kS) + " V");
                                    System.out.println("\tkV: " + formatter.format(kV) + " V/(rad/s) at MOTOR shaft");
                                    System.out.println("\t** NOTE: These values are for MOTOR velocity, not wheel velocity **");
                                    System.out.println("\t** Expected kV range: 0.020 to 0.030 for Kraken X60 **");
                                }));
    }

    // Class to hold state for wheel radius characterization data 
    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    public static Command wheelRadiusCharacterization(SwerveDriveSubsystem drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(0.05); // WHEEL_RADIUS_RAMP_RATE = 0.05;
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                    limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                    double speed = limiter.calculate(0.25); // WHEEL_RADIUS_MAX_VEL
                    drive.driveVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                        var rotation = drive.getRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                        double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius =
                            (state.gyroDelta * DriveConstants.kDriveBaseRadius) / wheelDelta;

                        NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println(
                            "********** Wheel Radius Characterization Results **********");
                        System.out.println(
                            "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println(
                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println(
                            "\tWheel Radius: "
                                + formatter.format(wheelRadius)
                                + " meters, "
                                + formatter.format(Units.metersToInches(wheelRadius))
                                + " inches");
                        })));
    }
}