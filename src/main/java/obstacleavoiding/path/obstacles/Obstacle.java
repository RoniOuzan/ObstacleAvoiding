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

    public boolean isLineInteracts(Translation2d pose1, Translation2d pose2) {
        for (int i = 0; i < this.getCorners().size(); i++) {
            Translation2d corner1 = this.getCorners().get(i);
            Translation2d corner2 = this.getCorners().get((i + 1) % this.getCorners().size());

            if (doSegmentsInteracts(pose1, pose2, corner1, corner2)) {
                return true;
            }
        }

        return false;
    }

    private boolean doSegmentsInteracts(Translation2d pose1, Translation2d pose2, Translation2d corner1, Translation2d corner2) {
        int o1 = orientation(pose1, pose2, corner1);
        int o2 = orientation(pose1, pose2, corner2);
        int o3 = orientation(corner1, corner2, pose1);
        int o4 = orientation(corner1, corner2, pose2);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        return (o1 == 0 && isPointOnSegment(pose1, pose2, corner1)) ||
                (o2 == 0 && isPointOnSegment(pose1, pose2, corner2)) ||
                (o3 == 0 && isPointOnSegment(corner1, corner2, pose1)) ||
                (o4 == 0 && isPointOnSegment(corner1, corner2, pose2));
    }

    private int orientation(Translation2d p, Translation2d q, Translation2d r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY());
        if (Math.abs(val) < 1e-9) {
            return 0; // Collinear
        } else if (val > 0) {
            return 1; // Clockwise orientation
        }
        return -1; // Counterclockwise orientation
    }

    private boolean isPointOnSegment(Translation2d p, Translation2d q, Translation2d r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }

    public boolean isPointAt(Translation2d translation2d) {
        int numPoints = this.getCorners().size();
        boolean isInside = false;
        for (int i = 0, j = numPoints - 1; i < numPoints; j = i++) {
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
