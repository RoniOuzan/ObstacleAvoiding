package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.util.Obstacle;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class ObstacleAvoiding {
    private final List<Obstacle> obstacles;
    private final double distanceThreshold;

    public ObstacleAvoiding(double distanceThreshold, List<Obstacle> obstacles) {
        this.obstacles = obstacles.stream().map(o -> o.getAlienatedObstacle(distanceThreshold)).collect(Collectors.toList());
        this.distanceThreshold = distanceThreshold;
    }

    public ObstacleAvoiding(double distanceThreshold, Obstacle... obstacles) {
        this(distanceThreshold, new ArrayList<>(Arrays.asList(obstacles)));
    }

    public List<Waypoint> generateWaypointsBinary(List<Waypoint> waypoints) {
        return generateWaypointsBinary(waypoints.toArray(new Waypoint[0]));
    }

    public List<Waypoint> generateWaypointsBinary(Waypoint... waypoints) {
        List<Waypoint> trajectory = new ArrayList<>(Arrays.asList(waypoints));
        for (int a = 0; a < 20 && getDistributingObstacles(trajectory).size() > 0; a++) {
//        while (getDistributingObstacles(trajectory).size() > 0) {
            int size = trajectory.size() - 1;
            for (int i = 0; i < size; i++) {
                Waypoint waypoint1 = trajectory.get(i);
                Waypoint waypoint2 = trajectory.get(i + 1);
                List<Obstacle> obstacles = this.getDistributingObstacle(waypoint1, waypoint2);
                if (obstacles.size() > 0) {
                    Translation2d middle = waypoint1.interpolate(waypoint2, 0.5);

                    Obstacle obstacle = this.getObstacle(middle);
                    if (obstacle != null) {
                        List<Translation2d> middles = new ArrayList<>();
                        for (int j = 0; j < obstacle.getCorners().size(); j++) {
                            double slopeMiddle = waypoint1.minus(waypoint2).getAngle().plus(Rotation2d.fromDegrees(90)).getTan();
                            Translation2d corner1 = obstacle.getCorners().get(j);
                            Translation2d corner2 = obstacle.getCorners().get((j + 1) % obstacle.getCorners().size());
                            double x, y;
                            if (corner1.getX() != corner2.getX()) {
                                // the slope of 2 corners of the polygon
                                double slopePol = corner1.minus(corner2).getAngle().getTan();

                                // an equation that finds the x of the 2 functions intersection
                                x = ((slopeMiddle * middle.getX()) - middle.getY() - (slopePol * corner1.getX()) + corner1.getY()) / (slopeMiddle - slopePol);
                            } else {
                                x = corner1.getX();
                            }
                            y = (slopeMiddle * x) - (slopeMiddle * middle.getX()) + middle.getY();

                            Translation2d newMiddle = new Translation2d(x, y);
                            Rotation2d angleFromMiddle = newMiddle.minus(obstacle.getCenter()).getAngle();
                            newMiddle = newMiddle.plus(new Translation2d(newMiddle.getDistance(middle) < 0.3 ? 0.5 : 0.2, angleFromMiddle));

                            if (this.getObstacle(newMiddle) != null)
                                continue;

                            middles.add(newMiddle);
                        }
                        middle = middles.stream().min(Comparator.comparing(middle::getDistance)).orElse(middle);
                    }

                    trajectory.add(i + 1, new Waypoint(middle, 90));
                }
            }
        }

        List<Waypoint> waypointList = new ArrayList<>(Arrays.asList(waypoints));
        for (int i = 0; i < trajectory.size(); i++) {
            for (int j = trajectory.size() - 1; j > i; j--) {
                if (this.getDistributingObstacle(trajectory.get(i), trajectory.get(j)).size() == 0) {
                    List<Integer> remove = new ArrayList<>();
                    for (int k = i + 1; k < j && !waypointList.contains(trajectory.get(k)); k++) {
                        remove.add(k);
                    }
                    remove.sort(Comparator.comparingInt(i2 -> i2));
                    for (int k = remove.size() - 1; k >= 0; k--) {
                        trajectory.remove((int) remove.get(k));
                    }
                    break;
                }
            }
        }

        return trajectory;
    }

    public List<Obstacle> getDistributingObstacles(List<Waypoint> waypoints) {
        List<Obstacle> obstacles = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            obstacles.addAll(getDistributingObstacle(waypoints.get(i), waypoints.get(i + 1)));
        }
        return obstacles;
    }

    public List<Obstacle> getDistributingObstacle(Translation2d initialPose, Translation2d finalPose) {
        List<Obstacle> obstacles = new ArrayList<>();
        for (Obstacle obstacle : this.obstacles) {
            if (this.isLineInsidePolygon(initialPose, finalPose, obstacle))
                obstacles.add(obstacle);
        }
        return obstacles;
    }

    private boolean isLineInsidePolygon(Translation2d pose1, Translation2d pose2, Obstacle obstacle) {
        int intersections = 0;

        for (int i = 0; i < obstacle.getCorners().size(); i++) {
            Translation2d p1 = obstacle.getCorners().get(i);
            Translation2d p2 = obstacle.getCorners().get((i + 1) % obstacle.getCorners().size());

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

    public Obstacle getObstacle(Translation2d translation2d) {
        for (Obstacle obstacle : this.obstacles) {
            if (isPointAtObstacle(translation2d, obstacle))
                return obstacle;
        }
        return null;
    }

    public boolean isPointAtObstacle(Translation2d translation2d, Obstacle obstacle) {
        int numPoints = obstacle.getCorners().size();
        int i, j;
        boolean isInside = false;

        for (i = 0, j = numPoints - 1; i < numPoints; j = i++) {
            Translation2d cornerI = obstacle.getCorners().get(i);
            Translation2d cornerJ = obstacle.getCorners().get(j);
            if ((cornerI.getY() > translation2d.getY()) != (cornerJ.getY() > translation2d.getY()) &&
                    (translation2d.getX() < (cornerJ.getX() - cornerI.getX()) * (translation2d.getY() - cornerI.getY()) / (cornerJ.getY() - cornerI.getY()) + cornerI.getX())) {
                isInside = !isInside;
            }
        }

        return isInside;
    }

    public List<Obstacle> getObstacles() {
        return obstacles;
    }

    public double getDistanceThreshold() {
        return distanceThreshold;
    }
}
