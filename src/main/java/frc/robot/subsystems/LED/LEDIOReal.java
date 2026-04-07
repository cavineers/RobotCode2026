package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * @brief Real hardware implementation of LED control using WPI AddressableLED.
 */
public class LEDIOReal implements LEDIO {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    
    /**
     * @brief Create a real LED IO instance.
     * @param port PWM port for the LED controller
     */
    public LEDIOReal(int port) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(55);
        led.setLength(buffer.getLength());
        setPattern(LEDPattern.solid(Color.kWhite));
        led.start();
    }
    
    @Override
    public void updateInputs(LEDIOInputs inputs) {
        // No inputs to update for LED subsystem
    }
    
    @Override
    public void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}
