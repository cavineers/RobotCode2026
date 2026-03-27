package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

/**
 * @brief Hardware interface for LED control.
 */
public interface LEDIO {
    
    @AutoLog
    public static class LEDIOInputs {
    }

    /**
     * @brief Update inputs from hardware.
     * @param inputs Input object to populate
     */
    public default void updateInputs(LEDIOInputs inputs) {}

    /**
     * @brief Set LED pattern.
     * @param pattern LEDPattern to display
     */
    public default void setPattern(LEDPattern pattern) {}
}
