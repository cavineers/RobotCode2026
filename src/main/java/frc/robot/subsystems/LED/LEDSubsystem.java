package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.LED.LEDConstants.*;

/**
 * @brief LED subsystem for controlling addressable LEDs.
 *
 * <p>Manages LED pattern based on the robot's alliance color (red or blue).
 */
public class LEDSubsystem extends SubsystemBase {
    
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
    
    /**
     * @brief Create an LED subsystem.
     * @param io Hardware interface implementation
     */
    public LEDSubsystem(LEDIO io) {
        this.io = io;
        updateAlliancePattern();
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);
        
        // Update LED pattern to match current alliance color
        updateAlliancePattern();
    }
    
    /**
     * @brief Update LED pattern based on current alliance.
     */
    private void updateAlliancePattern() {
        var alliance = DriverStation.getAlliance();
        
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                io.setPattern(kRedPattern);
            } else {
                io.setPattern(kBluePattern);
            }
        } else {
            // Default to white if alliance is not available
            io.setPattern(kDefaultPattern);
        }
    }
}
