package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ChargedUpField extends Field {
    public ChargedUpField() {
        super("ChargedUp", FieldType.MIRROR);
    }

    @Override
    protected List<Obstacle> generateBlueObstacles() {
        return Arrays.asList(
                new Obstacle("BlueRamp", Alliance.BLUE,
                        new Translation2d(2.951, 3.985),
                        new Translation2d(4.828, 3.985),
                        new Translation2d(4.828, 1.509),
                        new Translation2d(2.951, 1.509)),
                new Obstacle("BlueBarrier", Alliance.BLUE ,
                        new Translation2d(1.458, 5.506),
                        new Translation2d(3.363, 5.506),
                        new Translation2d(3.363, 5.474),
                        new Translation2d(1.458, 5.474)),
                new Obstacle("BlueGrid", Alliance.BLUE,
                        new Translation2d(0, 5.500),
                        new Translation2d(1.381, 5.500),
                        new Translation2d(1.381, 0),
                        new Translation2d(0, 0)));
    }

    @Override
    protected List<Obstacle> generateGeneralObstacles() {
        return new ArrayList<>();
    }
}
