package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.07;
    }

    /** Physical robot dimensions — update with measured values before competition. */
    public static final class RobotDimensions {
        /** Full width including bumpers, metres. TODO: measure actual robot. */
        public static final double kWidthMeters = 0.762;
        /** Full length including bumpers, metres. TODO: measure actual robot. */
        public static final double kLengthMeters = 0.762;
        /** Height of the bottom of the bumpers off the ground, metres. TODO: measure actual robot. */
        public static final double kBumperHeightMeters = 0.127;
    }
}
