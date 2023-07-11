package obstacleavoiding.path.waypoints;

import obstacleavoiding.math.geometry.Rotation2d;

public class NavigationWaypoint extends Waypoint {

    private double targetVelocity;

    public NavigationWaypoint(double x, double y, Rotation2d heading, RobotReferencePoint robotReferencePoint, double targetVelocity) {
        super(x, y, heading, robotReferencePoint);
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }
}
