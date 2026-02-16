package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Automatic shooter feedforward characterization using linear regression.
 * 
 */
public class ShooterCharacterizationCommand {
    
    /**
     * Create automatic feedforward characterization command.
     * Ramps voltage from 0 to 12V over ~12 seconds and calculates kS and kV.
     */
    public static Command feedforwardCharacterization(ShooterSubsystem shooter) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                            System.out.println("********** Shooter FF Characterization Started **********");
                        }),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * 1.0; // Ramp 1V per second
                            shooter.runCharacterization(voltage);
                            velocitySamples.add(shooter.getCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        shooter)
                        .withTimeout(12.0) // Run for 12 seconds (0-12V)

                        // When finished or cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    shooter.stop();
                                    
                                    int n = velocitySamples.size();
                                    if (n < 10) {
                                        System.out.println("Not enough data points collected!");
                                        return;
                                    }
                                    
                                    // Linear regression: V = kS + kV * velocity
                                    // Using least squares: y = a + bx
                                    double sumX = 0.0;  // Sum of velocities
                                    double sumY = 0.0;  // Sum of voltages
                                    double sumXY = 0.0; // Sum of velocity * voltage
                                    double sumX2 = 0.0; // Sum of velocity^2
                                    
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    
                                    // kS = y-intercept, kV = slope
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Shooter FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS) + " V");
                                    System.out.println("\tkV: " + formatter.format(kV) + " V/(rot/s)");
                                    System.out.println("\tData points: " + n);
                                    System.out.println("Update ShooterConstants.java with these values!");
                                    System.out.println("*********************************************************");
                                }));
    }
}
