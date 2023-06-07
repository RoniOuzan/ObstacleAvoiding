package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import java.util.function.BiFunction;
import java.util.function.Function;

public enum FieldType {
    SYMMETRY((t, b) -> new Translation2d(b.getX() - t.getX(), b.getY() - t.getY())),
    MIRROR((t, b) -> new Translation2d(b.getX() - t.getX(), t.getY()));

    private final BiFunction<Translation2d, Translation2d, Translation2d> convertToRedObstacle;

    FieldType(BiFunction<Translation2d, Translation2d, Translation2d> convertToRedObstacle) {
        this.convertToRedObstacle = convertToRedObstacle;
    }

    public Obstacle convertToRedObstacle(Obstacle obstacle, Translation2d bounds) {
        return new Obstacle(obstacle.getName(), Alliance.RED,
                obstacle.getCorners().stream().map(c -> this.convertToRedObstacle.apply(c, bounds)).toList());
    }
}
