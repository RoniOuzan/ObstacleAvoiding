package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;

import java.util.function.BiFunction;

public enum FieldType {
    SYMMETRY((t, b) -> new Translation2d(b.getX() - t.getX(), b.getY() - t.getY())),
    MIRROR((t, b) -> new Translation2d(b.getX() - t.getX(), t.getY()));

    private final BiFunction<Translation2d, Translation2d, Translation2d> convertToOtherAllianceObstacle;

    FieldType(BiFunction<Translation2d, Translation2d, Translation2d> convertToOtherAllianceObstacle) {
        this.convertToOtherAllianceObstacle = convertToOtherAllianceObstacle;
    }

    public Obstacle convertToOtherAllianceObstacle(Obstacle obstacle, Translation2d bounds) {
        return new Obstacle(obstacle.getName(), obstacle.getAlliance().getOther(),
                obstacle.getCorners().stream().map(c -> this.convertToOtherAllianceObstacle.apply(c, bounds)).toList());
    }
}
