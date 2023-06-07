package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class InfiniteRecharge extends Field {
    public InfiniteRecharge() {
        super("InfiniteRecharge", FieldType.SYMMETRY);
    }

    @Override
    protected List<Obstacle> generateAllianceObstacles() {
        return Arrays.asList(
                new Obstacle("UpThrusts", Alliance.BLUE,
                        new Translation2d(5.494, 5.030),
                        new Translation2d(5.764, 5.136),
                        new Translation2d(5.880, 4.885),
                        new Translation2d(5.609, 4.760)),
                new Obstacle("DownThrusts", Alliance.RED,
                        new Translation2d(6.942, 1.651),
                        new Translation2d(7.203, 1.747),
                        new Translation2d(7.318, 1.486),
                        new Translation2d(7.038, 1.419)),
                new Obstacle("Roleta", Alliance.BLUE,
                        new Translation2d(9.056, 6.729),
                        new Translation2d(9.800, 6.729),
                        new Translation2d(9.800, 6.672),
                        new Translation2d(9.056, 6.672))
        );
    }

    @Override
    protected List<Obstacle> generateGeneralObstacles() {
        return new ArrayList<>();
    }
}
