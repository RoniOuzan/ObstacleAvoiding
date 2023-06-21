package obstacleavoiding.path.robot;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.math.matrix.Matrix;

public class ModuleState {
    private final Translation2d location;
    private double velocity;
    private Rotation2d angle;

    public ModuleState(Translation2d location, double velocity, Rotation2d angle) {
        this.location = location;
        this.velocity = velocity;
        this.angle = angle;
    }

    public ModuleState(Translation2d location, double x, double y) {
        this(location, Math.hypot(x, y), Rotation2d.fromRadians(Math.atan2(y, x)));
    }

    public Translation2d getLocation() {
        return location;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public Rotation2d getAngle() {
        return angle;
    }

    public void setAngle(Rotation2d angle) {
        this.angle = angle;
    }

    public void set(double x, double y) {
        this.velocity = Math.hypot(x, y);
        this.angle = Rotation2d.fromRadians(Math.atan2(y, x));
    }

    @Override
    public String toString() {
        return String.format("ModuleState(Velocity: %.2f, Angle: %.2f)", this.velocity, this.angle.getDegrees());
    }
}
