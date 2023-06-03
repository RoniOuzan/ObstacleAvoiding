package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.Robot;

import java.util.function.BiFunction;
import java.util.function.Function;

public class Waypoint extends Translation2d {
    private final double heading;
    private final double movementAngle;

    public Waypoint(double x, double y, double heading, double movementAngle) {
        super(x, y);
        this.heading = heading;
        this.movementAngle = movementAngle;
    }

    public Waypoint(double x, double y) {
        this(x, y, 0, 0);
    }

    public Waypoint(double distance, Rotation2d angle, double heading, double movementAngle) {
        super(distance, angle);
        this.heading = heading;
        this.movementAngle = movementAngle;
    }

    public Waypoint(Translation2d translation2d, double heading, double movementAngle) {
        this(translation2d.getX(), translation2d.getY(), heading, movementAngle);
    }

    public Waypoint(Translation2d translation2d, Waypoint waypoint) {
        this(translation2d.getX(), translation2d.getY(), waypoint.heading, waypoint.movementAngle);
    }

    public Waypoint(Translation2d translation2d, double heading) {
        this(translation2d.getX(), translation2d.getY(), heading, 0);
    }

    public Waypoint(Translation2d translation2d) {
        this(translation2d.getX(), translation2d.getY(), 0, 0);
    }

    public double getHeading() {
        return heading;
    }

    public double getMovementAngle() {
        return movementAngle;
    }


    @Override
    public String toString() {
        return "(" + this.getX() + "," + this.getY() + "," + this.getHeading() + "," + this.getMovementAngle() + ")";
    }
}
