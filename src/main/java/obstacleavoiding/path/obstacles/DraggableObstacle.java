package main.java.obstacleavoiding.path.obstacles;

import main.java.obstacleavoiding.math.geometry.Translation2d;
import main.java.obstacleavoiding.path.util.Alliance;

import java.util.List;
import java.util.stream.Collectors;

public class DraggableObstacle extends Obstacle {
    public DraggableObstacle(String name, Alliance alliance, List<Translation2d> corners) {
        super(name, alliance, corners);
    }

    public DraggableObstacle(String name, Alliance alliance, Translation2d... corners) {
        super(name, alliance, corners);
    }

    public static List<Translation2d> getCornersOfObstacle(Translation2d center, Obstacle obstacle) {
        Translation2d oldCenter = obstacle.getCenter();
        Translation2d d = oldCenter.minus(center);
        return obstacle.getCorners().stream().map(c -> c.minus(d)).collect(Collectors.toList());
    }
}
