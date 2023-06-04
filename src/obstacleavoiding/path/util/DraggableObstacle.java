package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Translation2d;

import java.util.List;
import java.util.stream.Collectors;

public class DraggableObstacle extends Obstacle {
    public DraggableObstacle(String name, Alliance alliance, List<Translation2d> corners) {
        super(name, alliance, corners);
    }

    public DraggableObstacle(String name, Alliance alliance, Translation2d... corners) {
        super(name, alliance, corners);
    }

    public DraggableObstacle(Translation2d center, Obstacle obstacle) {
        super(obstacle.getName(), obstacle.getAlliance(), getCornersOfObstacle(center, obstacle));
    }

    private static List<Translation2d> getCornersOfObstacle(Translation2d center, Obstacle obstacle) {
        Translation2d oldCenter = obstacle.getCenter();
        Translation2d d = oldCenter.minus(center);
        return obstacle.getCorners().stream().map(c -> c.minus(d)).collect(Collectors.toList());
    }
}
