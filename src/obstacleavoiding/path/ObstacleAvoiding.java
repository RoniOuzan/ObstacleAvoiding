package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.util.Obstacle;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ObstacleAvoiding {
    private final List<Obstacle> obstacles;
    private final double distanceThreshold;

    public ObstacleAvoiding(double distanceThreshold, List<Obstacle> obstacles) {
        this.obstacles = obstacles;
        this.distanceThreshold = distanceThreshold;

        this.obstacles.forEach(o -> o.increaseArea(this.distanceThreshold));
    }

    public ObstacleAvoiding(double distanceThreshold, Obstacle... obstacles) {
        this(distanceThreshold, new ArrayList<>(Arrays.asList(obstacles)));
    }

    public boolean isThereAnObstacle(Waypoint startWaypoint, Waypoint finalWaypoint) {
        return this.obstacles.stream().anyMatch(o -> this.isLineInsidePolygon(startWaypoint, finalWaypoint, o));
    }

    private boolean isLineInsidePolygon(Translation2d pose1, Translation2d pose2, Obstacle obstacle) {
        int intersections = 0;

        for (int i = 0; i < obstacle.getCorners().size(); i++) {
            Translation2d p1 = obstacle.getCorners().get(i);
            Translation2d p2 = obstacle.getCorners().get((i + 1) % obstacle.getCorners().size());

            if (doSegmentsIntersect(pose1, pose2, p1, p2)) {
                intersections++;
            }
        }

        return intersections > 0;
    }

    private boolean doSegmentsIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        int o1 = orientation(p1, p2, q1);
        int o2 = orientation(p1, p2, q2);
        int o3 = orientation(q1, q2, p1);
        int o4 = orientation(q1, q2, p2);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        if (o1 == 0 && isPointOnSegment(p1, p2, q1))
            return true;
        if (o2 == 0 && isPointOnSegment(p1, p2, q2))
            return true;
        if (o3 == 0 && isPointOnSegment(q1, q2, p1))
            return true;
        return o4 == 0 && isPointOnSegment(q1, q2, p2);
    }

    private int orientation(Translation2d p, Translation2d q, Translation2d r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY());
        if (Math.abs(val) < 1e-9) {
            return 0; // Collinear
        } else if (val > 0) {
            return 1; // Clockwise orientation
        } else {
            return 2; // Counterclockwise orientation
        }
    }

    private boolean isPointOnSegment(Translation2d p, Translation2d q, Translation2d r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }

    public List<Obstacle> getObstacles() {
        return obstacles;
    }
}
