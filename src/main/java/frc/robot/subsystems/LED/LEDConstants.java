package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Percent;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;


/**
 * @brief Constants for LED subsystem.
 */
public class LEDConstants {
    /** PWM port for LED controller */
    public static final int kLEDPort = 0;
    
    /** LED pattern for red alliance */
    public static final LEDPattern kRedPattern = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(100));
    
    /** LED pattern for blue alliance */
    public static final LEDPattern kBluePattern = LEDPattern.solid(Color.kBlue);
    
    /** LED pattern for unknown alliance */
    public static final LEDPattern kDefaultPattern = LEDPattern.solid(Color.kWhite);
}
