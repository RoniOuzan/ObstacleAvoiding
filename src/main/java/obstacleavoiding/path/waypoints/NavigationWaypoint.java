package obstacleavoiding.path.waypoints;

public class NavigationWaypoint extends Waypoint {

    private double targetVelocity;

    public NavigationWaypoint(double x, double y, double heading, RobotReference robotReference, double targetVelocity) {
        super(x, y, heading, robotReference);
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }
}
