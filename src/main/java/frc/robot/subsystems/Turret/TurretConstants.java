package frc.robot.subsystems.Turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;

public class TurretConstants {

    //all currently placeholders, just getting setup for now!
    public static final boolean kTuningMode = true;


    public static final int kTurretCanID = 1; 
    public static final int kTurretLimitSwitchID = 0;
  
    public static final double kMaxAngle = Units.degreesToRadians(359.0); //359 degrees in radians
    public static final double kMinAngle = 0.0;

    

    public static final double kProportionalGainSpark = 0.0;// TODO: update with actual term
    public static final double kDerivativeTermSpark = 0.0;// TODO: update with actual term
    public static final double kIntegralTermSpark = 0.0; // TODO: update with actual term

    public static final double kProportionalTermSim = .1; // TODO: Update with actual term
    public static final double kDerivativeTermSim = .1; //TODO: Update with actual term

    public static final double kTurretGearRatio = 200.0 / 19.0; //prototype ratio
    public static final double kGearBacklash = Units.degreesToRadians(5); //protoype backlash

    public static boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double kCurrentLimit = 40.0;

  
}

