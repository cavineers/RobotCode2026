package frc.lib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Physics simulation for projectile motion with air resistance.
 * Uses RK4 (Runge-Kutta 4th order) integration for accurate trajectory simulation.
 */
public class ProjectilePhysics {
    
    private static final double G = 9.81; // m/s^2
    
    // Ball properties 
    private static final double BALL_MASS = 0.235; // kg (note foam ball)
    private static final double BALL_DIAMETER = 0.15; // meters (14 inches)
    private static final double BALL_RADIUS = BALL_DIAMETER / 2.0;
    private static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS * BALL_RADIUS;
    
    // Air resistance parameters
    private static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level
    private static final double DRAG_COEFFICIENT = 0.47; // sphere drag coefficient
    
    /**
     * State of a projectile in flight
     */
    public static class ProjectileState {
        public double x, y, z;        // position (m)
        public double vx, vy, vz;     // velocity (m/s)
        public double time;            // elapsed time (s)
        
        public ProjectileState(double x, double y, double z, double vx, double vy, double vz) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
            this.time = 0;
        }
        
        public Pose3d toPose3d() {
            return new Pose3d(x, y, z, new Rotation3d());
        }
        
        public Translation3d toTranslation3d() {
            return new Translation3d(x, y, z);
        }
    }
    
    /**
     * Simulate a projectile trajectory with air resistance using RK4 integration.
     * 
     * @param initialState Starting position and velocity
     * @param dt Time step for simulation (smaller = more accurate, suggest 0.01s)
     * @param maxTime Maximum simulation time (s)
     * @param groundHeight Height at which to stop simulation (m)
     * @return Array of states along the trajectory
     */
    public static ProjectileState[] simulateTrajectory(
            ProjectileState initialState, 
            double dt, 
            double maxTime,
            double groundHeight) {
        
        java.util.ArrayList<ProjectileState> trajectory = new java.util.ArrayList<>();
        ProjectileState current = copy(initialState);
        trajectory.add(copy(current));
        
        while (current.time < maxTime && current.z >= groundHeight) {
            current = rk4Step(current, dt);
            trajectory.add(copy(current));
        }
        
        return trajectory.toArray(new ProjectileState[0]);
    }
    
    /**
     * Simulate trajectory and return only the landing position.
     * More efficient than full trajectory for shot validation.
     */
    public static ProjectileState simulateLanding(
            ProjectileState initialState,
            double dt,
            double maxTime,
            double groundHeight) {
        
        ProjectileState current = copy(initialState);
        
        while (current.time < maxTime && current.z >= groundHeight) {
            current = rk4Step(current, dt);
        }
        
        return current;
    }
    
    /**
     * Calculate derivatives for RK4 integration.
     * Returns [dx/dt, dy/dt, dz/dt, dvx/dt, dvy/dt, dvz/dt]
     */
    private static double[] derivatives(ProjectileState state) {
        double[] deriv = new double[6];
        
        // Position derivatives (velocity)
        deriv[0] = state.vx;
        deriv[1] = state.vy;
        deriv[2] = state.vz;
        
        // Velocity magnitude
        double v = Math.sqrt(state.vx * state.vx + state.vy * state.vy + state.vz * state.vz);
        
        // Drag force magnitude: F_drag = 0.5 * ρ * v² * C_d * A
        double dragForce = 0.5 * AIR_DENSITY * v * v * DRAG_COEFFICIENT * BALL_CROSS_SECTION;
        
        // Acceleration from drag (opposite to velocity direction)
        double dragAccel = dragForce / BALL_MASS;
        
        // Velocity derivatives (acceleration)
        if (v > 0.001) { // avoid division by zero
            deriv[3] = -(dragAccel * state.vx / v); // ax = -drag * vx/|v|
            deriv[4] = -(dragAccel * state.vy / v); // ay = -drag * vy/|v|
            deriv[5] = -(dragAccel * state.vz / v) - G; // az = -drag * vz/|v| - g
        } else {
            deriv[3] = 0;
            deriv[4] = 0;
            deriv[5] = -G;
        }
        
        return deriv;
    }
    
    /**
     * Runge-Kutta 4th order integration step
     */
    private static ProjectileState rk4Step(ProjectileState state, double dt) {
        // k1 = f(t, y)
        double[] k1 = derivatives(state);
        
        // k2 = f(t + dt/2, y + k1*dt/2)
        ProjectileState temp = new ProjectileState(
            state.x + k1[0] * dt / 2,
            state.y + k1[1] * dt / 2,
            state.z + k1[2] * dt / 2,
            state.vx + k1[3] * dt / 2,
            state.vy + k1[4] * dt / 2,
            state.vz + k1[5] * dt / 2
        );
        double[] k2 = derivatives(temp);
        
        // k3 = f(t + dt/2, y + k2*dt/2)
        temp = new ProjectileState(
            state.x + k2[0] * dt / 2,
            state.y + k2[1] * dt / 2,
            state.z + k2[2] * dt / 2,
            state.vx + k2[3] * dt / 2,
            state.vy + k2[4] * dt / 2,
            state.vz + k2[5] * dt / 2
        );
        double[] k3 = derivatives(temp);
        
        // k4 = f(t + dt, y + k3*dt)
        temp = new ProjectileState(
            state.x + k3[0] * dt,
            state.y + k3[1] * dt,
            state.z + k3[2] * dt,
            state.vx + k3[3] * dt,
            state.vy + k3[4] * dt,
            state.vz + k3[5] * dt
        );
        double[] k4 = derivatives(temp);
        
        // y_next = y + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        ProjectileState next = new ProjectileState(
            state.x + (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) * dt / 6,
            state.y + (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) * dt / 6,
            state.z + (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) * dt / 6,
            state.vx + (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) * dt / 6,
            state.vy + (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]) * dt / 6,
            state.vz + (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]) * dt / 6
        );
        next.time = state.time + dt;
        
        return next;
    }
    
    private static ProjectileState copy(ProjectileState state) {
        ProjectileState copy = new ProjectileState(state.x, state.y, state.z, state.vx, state.vy, state.vz);
        copy.time = state.time;
        return copy;
    }
    
    /**
     * Create initial state from shot parameters.
     * 
     * @param shooterPose Robot/shooter position
     * @param yawRad Horizontal angle (radians)
     * @param pitchRad Hood/launch angle (radians)
     * @param velocity Exit velocity (m/s)
     * @return Initial projectile state
     */
    public static ProjectileState createInitialState(
            Pose3d shooterPose,
            double yawRad,
            double pitchRad,
            double velocity) {
        
        // Decompose velocity into components
        double vx = velocity * Math.cos(pitchRad) * Math.cos(yawRad);
        double vy = velocity * Math.cos(pitchRad) * Math.sin(yawRad);
        double vz = velocity * Math.sin(pitchRad);
        
        return new ProjectileState(
            shooterPose.getX(),
            shooterPose.getY(),
            shooterPose.getZ(),
            vx, vy, vz
        );
    }
    
    /**
     * Create initial state from shot parameters, including robot velocity.
     * This accounts for the projectile inheriting the robot's velocity.
     * 
     * @param shooterPose Robot/shooter position
     * @param robotVx Robot velocity in X (m/s)
     * @param robotVy Robot velocity in Y (m/s)
     * @param yawRad Horizontal angle (radians)
     * @param pitchRad Hood/launch angle (radians)
     * @param velocity Exit velocity (m/s)
     * @return Initial projectile state with robot velocity added
     */
    public static ProjectileState createInitialStateWithRobotVelocity(
            Pose3d shooterPose,
            double robotVx,
            double robotVy,
            double yawRad,
            double pitchRad,
            double velocity) {
        
        // Decompose exit velocity into components
        double exitVx = velocity * Math.cos(pitchRad) * Math.cos(yawRad);
        double exitVy = velocity * Math.cos(pitchRad) * Math.sin(yawRad);
        double exitVz = velocity * Math.sin(pitchRad);
        
        // Add robot velocity to projectile exit velocity
        return new ProjectileState(
            shooterPose.getX(),
            shooterPose.getY(),
            shooterPose.getZ(),
            exitVx + robotVx,  // Projectile inherits robot's X velocity
            exitVy + robotVy,  // Projectile inherits robot's Y velocity
            exitVz             // Z velocity is only from the shot
        );
    }
}
