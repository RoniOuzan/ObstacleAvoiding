package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;

public class WaypointAutoHeading extends Waypoint {
    public WaypointAutoHeading(double x, double y, RobotReference robotReference) {
        super(x, y, robotReference);
    }

    public WaypointAutoHeading(double distance, Rotation2d angle, RobotReference robotReference) {
        super(distance, angle, 0, robotReference);
    }

    public WaypointAutoHeading(Translation2d translation2d, RobotReference robotReference) {
        super(translation2d, robotReference);
    }
}
