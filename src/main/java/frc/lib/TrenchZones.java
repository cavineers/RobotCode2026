package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrenchZones {

    private final double minX, maxX, minY, maxY;

    public TrenchZones(double minX, double maxX,
                     double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    // Check if robot is inside the zone
    public boolean contains(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();

        return x >= minX && x <= maxX
            && y >= minY && y <= maxY;
    }

    // Corners used for drawing on Field2d
    public Pose2d[] getCorners() {
        return new Pose2d[] {
            new Pose2d(minX, minY, new Rotation2d()),
            new Pose2d(maxX, minY, new Rotation2d()),
            new Pose2d(maxX, maxY, new Rotation2d()),
            new Pose2d(minX, maxY, new Rotation2d()),
            new Pose2d(minX, minY, new Rotation2d()) // close shape
        };
    }
}