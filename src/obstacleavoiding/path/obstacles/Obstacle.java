package obstacleavoiding.path.obstacles;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Obstacle {
    private final String name;
    private final Alliance alliance;
    private List<Translation2d> corners;

    public Obstacle(String name, Alliance alliance, List<Translation2d> corners) {
        this.name = name;
        this.alliance = alliance;
        this.corners = corners;
    }

    public Obstacle(String name, Alliance alliance, Translation2d... corners) {
        this(name, alliance, new ArrayList<>(Arrays.asList(corners)));
    }

    public List<Translation2d> getCorners() {
        return corners;
    }

    public void setCorners(List<Translation2d> corners) {
        this.corners = corners;
    }

    public Alliance getAlliance() {
        return alliance;
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

    public boolean isLineInside(Translation2d pose1, Translation2d pose2) {
        int intersections = 0;

        for (int i = 0; i < this.getCorners().size(); i++) {
            Translation2d p1 = this.getCorners().get(i);
            Translation2d p2 = this.getCorners().get((i + 1) % this.getCorners().size());

            if (doSegmentsIntersect(pose1, pose2, p1, p2)) {
                intersections++;
            }
        }

        return intersections > 0;
    }

    private boolean doSegmentsIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        int o1 = orientation(p1, p2, q1);
        int o2 = orientation(p1, p2, q2);
        int o3 = orientation(q1, q2, p1);
        int o4 = orientation(q1, q2, p2);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        if (o1 == 0 && isPointOnSegment(p1, p2, q1))
            return true;
        if (o2 == 0 && isPointOnSegment(p1, p2, q2))
            return true;
        if (o3 == 0 && isPointOnSegment(q1, q2, p1))
            return true;
        return o4 == 0 && isPointOnSegment(q1, q2, p2);
    }

    private int orientation(Translation2d p, Translation2d q, Translation2d r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY());
        if (Math.abs(val) < 1e-9) {
            return 0; // Collinear
        } else if (val > 0) {
            return 1; // Clockwise orientation
        } else {
            return 2; // Counterclockwise orientation
        }
    }

    private boolean isPointOnSegment(Translation2d p, Translation2d q, Translation2d r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }

    public boolean isPointAt(Translation2d translation2d) {
        int numPoints = this.getCorners().size();
        int i, j;
        boolean isInside = false;
        for (i = 0, j = numPoints - 1; i < numPoints; j = i++) {
            Translation2d cornerI = this.getCorners().get(i);
            Translation2d cornerJ = this.getCorners().get(j);
            if ((cornerI.getY() > translation2d.getY()) != (cornerJ.getY() > translation2d.getY()) &&
                    (translation2d.getX() < (cornerJ.getX() - cornerI.getX()) * (translation2d.getY() - cornerI.getY()) / (cornerJ.getY() - cornerI.getY()) + cornerI.getX())) {
                isInside = !isInside;
            }
        }

        return isInside;
    }

    @Override
    public String toString() {
        return this.name;
    }
}
