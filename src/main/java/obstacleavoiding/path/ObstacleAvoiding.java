package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Bounds;
import obstacleavoiding.path.util.Waypoint;
import obstacleavoiding.util.Entry;

import java.util.*;

/**
 * This class creates a path that not crashing given obstacles with a given distance from the obstacles
 * It creates the path by every two points that crashes an obstacle creating a point between them and
 * moves it outside the obstacle and keeps like that until we have a some path and then chooses the
 * shortest out of them.
 */
public class ObstacleAvoiding {
    private final List<Obstacle> obstacles;
    private final double distanceThreshold;

    private final Bounds bounds;

    private boolean isFiltering = true;

    /**
     * Constructs a ObstacleAvoiding with the provided values.
     *
     * @param distanceThreshold the minimum distance from the center of the robot to the obstacle
     * @param bounds the field bounds to not drive to outside the field
     * @param obstacles the obstacles of the field that it will avoid
     */
    public ObstacleAvoiding(double distanceThreshold, Bounds bounds, List<Obstacle> obstacles) {
        this.distanceThreshold = distanceThreshold;
        this.bounds = bounds;
        this.obstacles = obstacles;
    }

    /**
     * Constructs a ObstacleAvoiding with the provided values.
     *
     * @param distanceThreshold the minimum distance from the center of the robot to the obstacle
     * @param bounds the field bounds to not drive to outside the field
     * @param obstacles the obstacles of the field that it will avoid
     */
    public ObstacleAvoiding(double distanceThreshold, Bounds bounds, Obstacle... obstacles) {
        this(distanceThreshold, bounds, new ArrayList<>(Arrays.asList(obstacles)));
    }

    /**
     * Generates a list of waypoints that represents the points that we need to drive
     * in order to avoid the given obstacles.
     * The path will save the heading and the RobotReference.
     *
     * @param waypoints the waypoints that represents the path that we need to change in order
     *                  drive for not crashing the obstacles.
     * @return the list of waypoints that represents the path that avoided the obstacles.
     */
    public List<Waypoint> generateWaypointsBinary(List<Waypoint> waypoints) {
        if (waypoints.stream().anyMatch(w -> this.getObstacle(w) != null))
            return new ArrayList<>(waypoints);

        long started = System.nanoTime();
        Map<List<Waypoint>, Integer> trajectories = new HashMap<>();
        for (int b = 0; b < 4; b++) {
            printStateFinished("Started " + b, started);
            List<Waypoint> trajectory = new ArrayList<>(waypoints);
            boolean split = false;
            while (getDistributingObstacles(trajectory).size() > 0 && trajectory.size() <= 30) {
                int size = trajectory.size() - 1;
                for (int i = 0; i < size; i++) {
                    Waypoint waypoint1 = trajectory.get(i);
                    Waypoint waypoint2 = trajectory.get(i + 1);

                    List<Obstacle> obstacles = this.getDistributingObstacle(waypoint1, waypoint2);
                    if (obstacles.size() > 0) {
                        Waypoint middle = new Waypoint(waypoint1.interpolate(waypoint2, 0.5), 0, Waypoint.RobotReference.CENTER);

                        Obstacle obstacle = this.getObstacle(middle);
                        if (obstacle != null) {
                            List<Waypoint> middles = new ArrayList<>();

                            for (int j = 0; j < obstacle.getCorners().size(); j++) {
                                double slopeMiddle = waypoint1.minus(waypoint2).getAngle().plus(Rotation2d.fromDegrees(90)).getTan();

                                Translation2d corner1 = obstacle.getCorners().get(j);
                                Translation2d corner2 = obstacle.getCorners().get((j + 1) % obstacle.getCorners().size());

                                double x;
                                if (corner1.getX() != corner2.getX()) {
                                    double slopePol = corner1.minus(corner2).getAngle().getTan(); // the slope of 2 corners of the polygon
                                    x = ((slopeMiddle * middle.getX()) - middle.getY() - (slopePol * corner1.getX()) + corner1.getY()) / (slopeMiddle - slopePol); // an equation that finds the x of the 2 functions intersection
                                } else {
                                    x = corner1.getX();
                                }
                                double y = (slopeMiddle * x) - (slopeMiddle * middle.getX()) + middle.getY();

                                Translation2d newMiddle = new Translation2d(x, y);
                                Rotation2d angleFromMiddle = newMiddle.minus(middle).getAngle();
                                newMiddle = newMiddle.plus(new Translation2d(isCloseToCorner(newMiddle, obstacle, 0.3) ? 0.4 : 0.25, angleFromMiddle));

                                int escapeTimes = 0;
                                while (this.getObstacle(newMiddle) != null) {
                                    newMiddle = newMiddle.plus(new Translation2d(0.1, angleFromMiddle));
                                    escapeTimes++;

                                    if (escapeTimes >= 20) break;
                                }

                                if (this.getObstacle(newMiddle) != null || !this.bounds.isInOfBounds(newMiddle)) {
                                    escapeTimes = 0;
                                    newMiddle = newMiddle.minus(new Translation2d(5, angleFromMiddle));
                                    while (this.getObstacle(newMiddle) != null) {
                                        newMiddle = newMiddle.minus(new Translation2d(0.1, angleFromMiddle));
                                        escapeTimes++;

                                        if (escapeTimes >= 20) break;
                                    }
                                    if (escapeTimes == 20) continue;
                                }

                                if (!this.bounds.isInOfBounds(newMiddle))
                                    continue;

                                middles.add(new Waypoint(newMiddle, 0, Waypoint.RobotReference.CENTER));
                            }

                            middles = middles.stream().sorted(Comparator.comparing(middle::getDistance)).toList();
                            if (!split && trajectory.size() < 10) {
                                if (b < middles.size())
                                    middle = middles.get(b);

                                split = true;
                            } else if (middles.size() > 0) {
                                middle = middles.get(0);
                            }
                        }

                        trajectory.add(i + 1, middle);
                    }
                }
            }

            printStateFinished("Filter " + b + ", " + trajectory.size(), started);
            if (trajectory.size() > 30)
                continue;
            if (this.isFiltering) {
                for (int i = 0; i < trajectory.size(); i++) {
                    for (int j = trajectory.size() - 1; j > i; j--) {
                        if (this.getDistributingObstacle(trajectory.get(i), trajectory.get(j)).size() == 0) {
                            List<Integer> remove = new ArrayList<>();
                            for (int k = i + 1; k < j && !waypoints.contains(trajectory.get(k)); k++) {
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
            }

            if (this.getDistributingObstacles(trajectory).size() == 0)
                trajectories.put(trajectory, b);
        }

        Entry<List<Waypoint>, Integer> result = trajectories.entrySet().stream().map(e -> new Entry<>(e.getKey(), e.getValue())).min(Comparator.comparing(e -> getPathDistance(e.a()))).orElse(new Entry<>(waypoints, -1));
        printStateFinished("Finished " + result.b(), started);
        return result.a();
    }

    private void printStateFinished(String text, long started) {
        System.out.println(text + ": " + (System.nanoTime() - started) / 1_000_000_000d);
    }

    private boolean isCloseToCorner(Translation2d translation2d, Obstacle obstacle, double threshold) {
        for (Translation2d corner : obstacle.getCorners()) {
            if (translation2d.getDistance(corner) <= threshold) {
                return true;
            }
        }

        for (int i = 0; i < 360; i += 90) {
            if (this.getObstacle(translation2d.plus(new Translation2d(0.3, Rotation2d.fromDegrees(i)))) != null)
                return true;
        }

        return false;
    }

    /**
     * Finds the distributing obstacles of a path.
     *
     * @param waypoints the waypoints that represents the path
     * @return the list of obstacles that distributing the path.
     *         if none found it will return an empty path.
     */
    public List<Obstacle> getDistributingObstacles(List<Waypoint> waypoints) {
        List<Obstacle> obstacles = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            obstacles.addAll(getDistributingObstacle(waypoints.get(i), waypoints.get(i + 1)));
        }
        return obstacles;
    }

    /**
     * Returns the distributing obstacles between 2 points.
     *
     * @param initialPose the first point
     * @param finalPose the second point
     * @return the list of obstacles that distributing the path.
     *         if none found it will return an empty path.
     */
    public List<Obstacle> getDistributingObstacle(Translation2d initialPose, Translation2d finalPose) {
        return this.obstacles.parallelStream().map(o -> o.getExtendedObstacle(this.distanceThreshold)).filter(o -> o.isLineInside(initialPose, finalPose)).toList();
    }

    /**
     * Find an obstacle at the given point
     *
     * @param translation2d the point to find the obstacle
     * @return the obstacle in the given point.
     *         if none found, it will return null.
     */
    public Obstacle getObstacle(Translation2d translation2d) {
        return this.obstacles.parallelStream().map(o -> o.getExtendedObstacle(this.distanceThreshold)).filter(o -> o.isPointAt(translation2d)).findFirst().orElse(null);
    }

    /**
     * Filtering means it will remove any useless point the path created what makes it more efficient and shorter.
     * If filtering is on false, it means the path will have a lot more points and most of the time
     * not be efficient as it could be.
     *
     * @return the current state of the filter
     */
    public boolean isFiltering() {
        return isFiltering;
    }

    /**
     * Changes the state of the filter.
     * Filtering means it will remove any useless point the path created what makes it more efficient and shorter.
     * If filtering is on false, it means the path will have a lot more points and most of the time
     * not be efficient as it could be.
     *
     * @param filtering the state of the filter
     */
    public void setFiltering(boolean filtering) {
        isFiltering = filtering;
    }

    /**
     * @return the obstacles that was given to this instance
     */
    public List<Obstacle> getObstacles() {
        return obstacles;
    }

    /**
     * @return the distance from the center of the robot to the obstacle that it keeps every path
     */
    public double getDistanceThreshold() {
        return distanceThreshold;
    }

    private static double getPathDistance(List<Waypoint> waypoints) {
        double distance = 0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            distance += waypoints.get(i).getDistance(waypoints.get(i + 1));
        }
        return distance;
    }
}
