package obstacleavoiding.path.waypoints;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;

public class WaypointAutoHeading extends Waypoint {
    public WaypointAutoHeading(double x, double y) {
        super(x, y, RobotReference.CENTER);
    }

    public WaypointAutoHeading(double distance, Rotation2d angle) {
        super(distance, angle, 0, RobotReference.CENTER);
    }

    public WaypointAutoHeading(Translation2d translation2d) {
        super(translation2d, RobotReference.CENTER);
    }
}
