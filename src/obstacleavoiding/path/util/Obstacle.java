package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;

public class Obstacle {
    private final UUID uuid;

    private final String name;
    private final Alliance alliance;
    private List<Translation2d> corners;

    protected Obstacle(UUID uuid, String name, Alliance alliance, List<Translation2d> corners) {
        this.uuid = uuid;
        this.name = name;
        this.alliance = alliance;
        this.corners = corners;
    }

    public Obstacle(String name, Alliance alliance, List<Translation2d> corners) {
        this(UUID.randomUUID(), name, alliance, corners);
    }

    public Obstacle(String name, Alliance alliance, Translation2d... corners) {
        this(name, alliance, new ArrayList<>(Arrays.asList(corners)));
    }

    public List<Translation2d> getCorners() {
        return corners;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public String getName() {
        return name;
    }

    public UUID getUuid() {
        return uuid;
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
        Obstacle obstacle = new Obstacle(this.name, this.alliance, new ArrayList<>(this.corners));
        obstacle.alienateCorners(amount);
        return obstacle;
    }

    public Obstacle getExtendedObstacle(double amount) {
        amount = Math.hypot(amount, amount);
        List<Translation2d> corners = new ArrayList<>(this.corners);
        for (int i = 0; i < corners.size(); i++) {
            Translation2d previousCorner = this.corners.get((i + corners.size() - 1) % corners.size());
            Translation2d corner = this.corners.get(i);
            Translation2d nextCorner = this.corners.get((i + 1) % corners.size());

            Translation2d previousVector = corner.minus(previousCorner);
            previousVector = previousVector.div(previousVector.getNorm());

            Translation2d nextVector = corner.minus(nextCorner);
            nextVector = nextVector.div(nextVector.getNorm());

            corners.set(i, corner.plus(new Translation2d(amount, previousVector.plus(nextVector).getAngle())));
        }
        return new Obstacle(this.name, this.alliance, corners);
    }

    @Override
    public String toString() {
        return this.name;
    }
}
