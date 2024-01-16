package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CrescendoField extends Field {
    public CrescendoField() {
        super("Crescendo", FieldType.MIRROR);
    }

    @Override
    protected List<Obstacle> generateAllianceObstacles() {
        return Arrays.asList(
                new Obstacle("RedLowerTruss", Alliance.RED,
                        new Translation2d(10.872, 2.974),
                        new Translation2d(11.143, 2.81),
                        new Translation2d(11.007, 2.578),
                        new Translation2d(10.737, 2.713)),
                new Obstacle("RedUpperTruss", Alliance.RED ,
                        new Translation2d(10.998, 5.504),
                        new Translation2d(11.133, 5.262),
                        new Translation2d(10.853, 5.117),
                        new Translation2d(10.727, 5.33)),
                new Obstacle("RedPodium", Alliance.RED,
                        new Translation2d(13.315, 4.21),
                        new Translation2d(13.315, 3.882),
                        new Translation2d(13.025, 3.882),
                        new Translation2d(13.025, 4.21)),
                new Obstacle("RedSubroofer", Alliance.RED,
                        new Translation2d(16.54, 6.489),
                        new Translation2d(15.623, 5.977),
                        new Translation2d(15.613, 4.934),
                        new Translation2d(16.54, 4.461))
        );
    }

    @Override
    protected List<Obstacle> generateGeneralObstacles() {
        return new ArrayList<>();
    }
}
