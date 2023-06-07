package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RapidReactField extends Field {
    public RapidReactField() {
        super("RapidReact", FieldType.SYMMETRY);
    }

    @Override
    protected List<Obstacle> generateBlueObstacles() {
        return Arrays.asList(
                new Obstacle("ThrustsDR", Alliance.BLUE,
                        new Translation2d(2.877, 5.532),
                        new Translation2d(3.253, 5.532),
                        new Translation2d(3.253, 5.223),
                        new Translation2d(2.877, 5.223)),
                new Obstacle("ThrustsUR", Alliance.BLUE,
                        new Translation2d(2.935, 7.946),
                        new Translation2d(3.234, 7.946),
                        new Translation2d(3.234, 7.637),
                        new Translation2d(2.935, 7.637)),
                new Obstacle("ThrustsUL", Alliance.BLUE,
                        new Translation2d(0.135, 7.946),
                        new Translation2d(0.463, 7.946),
                        new Translation2d(0.463, 7.656),
                        new Translation2d(0.135, 7.656)),
                new Obstacle("ThrustsDL", Alliance.BLUE,
                        new Translation2d(0.135, 5.532),
                        new Translation2d(0.463, 5.532),
                        new Translation2d(0.463, 5.223),
                        new Translation2d(0.135, 5.223))
        );
    }

    @Override
    protected List<Obstacle> generateGeneralObstacles() {
        return Arrays.asList(
                new Obstacle("HubMiddle", Alliance.NONE,
                        new Translation2d(8.023, 5.020),
                        new Translation2d(9.095, 4.625),
                        new Translation2d(9.307, 4.267),
                        new Translation2d(8.892, 3.218),
                        new Translation2d(8.525, 3.012),
                        new Translation2d(7.444, 3.437),
                        new Translation2d(7.260, 3.804),
                        new Translation2d(7.685, 4.866)),
                new Obstacle("HubUp", Alliance.NONE,
                        new Translation2d(7.541, 5.185),
                        new Translation2d(7.898, 5.300),
                        new Translation2d(8.023, 5.020),
                        new Translation2d(7.685, 4.866)),
                new Obstacle("HubRight", Alliance.NONE,
                        new Translation2d(9.095, 4.625),
                        new Translation2d(9.443, 4.750),
                        new Translation2d(9.626, 4.402),
                        new Translation2d(9.307, 4.267)),
                new Obstacle("HubDown", Alliance.NONE,
                        new Translation2d(8.892, 3.218),
                        new Translation2d(9.027, 2.887),
                        new Translation2d(8.641, 2.702),
                        new Translation2d(8.525, 3.012)),
                new Obstacle("HubLeft", Alliance.NONE,
                        new Translation2d(7.444, 3.437),
                        new Translation2d(7.096, 3.302),
                        new Translation2d(6.952, 3.659),
                        new Translation2d(7.260, 3.804))
        );
    }
}
