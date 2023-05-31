package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Obstacle {
    private final String name;
    private List<Translation2d> corners;

    public Obstacle(String name, List<Translation2d> corners) {
        this.name = name;
        this.corners = corners;
    }

    public Obstacle(String name, Translation2d... corners) {
        this(name, new ArrayList<>(Arrays.asList(corners)));
    }

    public List<Translation2d> getCorners() {
        return corners;
    }

    public String getName() {
        return name;
    }

    public Translation2d getCenter() {
        return new Translation2d(
                this.corners.stream().mapToDouble(Translation2d::getX).average().orElse(0),
                this.corners.stream().mapToDouble(Translation2d::getY).average().orElse(0));
    }

    public double getDirection(int index) {
        return this.corners.get(index).minus(this.getCenter()).getAngle().getDegrees();
    }

    public Translation2d alienateCorner(Translation2d corner, double amount) {
        double angle = corner.minus(this.getCenter()).getAngle().getRadians();
        return corner.plus(new Translation2d(
                Math.signum(Math.cos(angle)) * amount, Math.signum(Math.sin(angle)) * amount));
    }

    public void alienateCorners(double amount) {
        List<Translation2d> corners = new ArrayList<>();
        for (Translation2d corner : this.corners) {
            corners.add(this.alienateCorner(corner, amount));
        }
        this.corners = corners;
    }

    public Obstacle getAlienatedObstacle(double amount) {
        Obstacle obstacle = new Obstacle(this.getName(), new ArrayList<>(this.getCorners()));
        obstacle.alienateCorners(amount);
        return obstacle;
    }

    @Override
    public String toString() {
        return this.name;
    }
}
