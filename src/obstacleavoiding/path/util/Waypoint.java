package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.Robot;

import java.util.function.BiFunction;
import java.util.function.Function;

public class Waypoint extends Translation2d {
    private final double heading;
    private final double movementAngle;

    private final BiFunction<Robot, Waypoint, Boolean> passedWaypoint;

    public Waypoint(double x, double y, double heading, double movementAngle, BiFunction<Robot, Waypoint, Boolean> passedWaypoint) {
        super(x, y);
        this.heading = heading;
        this.movementAngle = movementAngle;
        this.passedWaypoint = passedWaypoint;
    }

    public Waypoint(double x, double y, BiFunction<Robot, Waypoint, Boolean> passedWaypoint) {
        this(x, y, 0, 0, passedWaypoint);
    }

    public Waypoint(double distance, Rotation2d angle, double heading, double movementAngle, BiFunction<Robot, Waypoint, Boolean> passedWaypoint) {
        super(distance, angle);
        this.heading = heading;
        this.movementAngle = movementAngle;
        this.passedWaypoint = passedWaypoint;
    }

    public Waypoint(Translation2d translation2d, double heading, double movementAngle, BiFunction<Robot, Waypoint, Boolean> passedWaypoint) {
        this(translation2d.getX(), translation2d.getY(), heading, movementAngle, passedWaypoint);
    }

    public Waypoint(Translation2d translation2d, Waypoint waypoint) {
        this(translation2d.getX(), translation2d.getY(), waypoint.heading, waypoint.movementAngle, waypoint.passedWaypoint);
    }

    public boolean isPassedWaypoint(Robot robot) {
        return this.passedWaypoint.apply(robot, this);
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
