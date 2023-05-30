package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Obstacle {
    private final List<Translation2d> corners;

    public Obstacle(List<Translation2d> corners) {
        this.corners = corners;
    }

    public Obstacle(Translation2d... corners) {
        this(new ArrayList<>(Arrays.asList(corners)));
    }

    public List<Translation2d> getCorners() {
        return corners;
    }

    public Translation2d getCenter() {
        return new Translation2d(
                this.corners.stream().mapToDouble(Translation2d::getX).average().orElse(0),
                this.corners.stream().mapToDouble(Translation2d::getY).average().orElse(0));
    }

    public void increaseArea(double amount) {
        Translation2d center = getCenter();
        for (int i = 0; i < this.corners.size(); i++) {
            Translation2d corner = this.corners.get(i);
            double angle = corner.minus(center).getAngle().getDegrees();
            angle = Math.toRadians(Math.round(angle / 45) * 45d);

            this.corners.set(i, corner.plus(new Translation2d(Math.signum(Math.cos(angle)) * amount, Math.signum(Math.sin(angle)) * amount)));
        }
    }
}
