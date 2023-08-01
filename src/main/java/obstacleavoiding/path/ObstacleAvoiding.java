package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Bounds;
import obstacleavoiding.path.waypoints.Waypoint;
import obstacleavoiding.path.waypoints.WaypointAutoHeading;

import java.util.*;
import java.util.stream.Collectors;

/**
 * This class creates a path that not crashing given obstacles with a given distance from the obstacles
 * It creates the path by every two points that crashes an obstacle creating a point between them and
 * moves it outside the obstacle and keeps like that until we have a some path and then chooses the
 * shortest out of them.
 */
public class ObstacleAvoiding {

    private static final int MAX_PATH_LENGTH = 30;

    private List<Obstacle> obstacles;
    private List<Obstacle> extendedObstacles;
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
        this.extendedObstacles = obstacles.parallelStream().map(o -> o.getExtendedObstacle(this.distanceThreshold)).collect(Collectors.toList());
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
        if (waypoints.stream().map(Waypoint::getReferencedPosition).anyMatch(w -> this.getObstacle(w) != null) ||
                this.isPathSafe(waypoints))
            return new ArrayList<>(waypoints);

        long started = System.nanoTime();
        printStateFinished("Started ", started);
        List<Waypoint> trajectory = new ArrayList<>(waypoints);

        Translation2d waypoint1, waypoint2, middle, corner1, corner2;
        Obstacle obstacle;

        while (trajectory.size() <= MAX_PATH_LENGTH && !isPathSafe(trajectory)) {
            int size = trajectory.size() - 1;
            for (int i = 0; i < size; i++) {
                waypoint1 = trajectory.get(i).getReferencedPosition();
                waypoint2 = trajectory.get(i + 1).getReferencedPosition();

                if (this.isObstacleDistributing(waypoint1, waypoint2)) {
                    middle = waypoint1.interpolate(waypoint2, 0.5);

                    obstacle = this.getObstacle(middle);
                    if (obstacle != null) {
                        List<Translation2d> middles = new ArrayList<>();

                        for (int j = 0; j < obstacle.getCorners().size(); j++) {
                            double slopeMiddle = waypoint1.minus(waypoint2).getAngle().plus(Rotation2d.fromDegrees(90)).getTan();

                            corner1 = obstacle.getCorners().get(j);
                            corner2 = obstacle.getCorners().get((j + 1) % obstacle.getCorners().size());

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
                                newMiddle = newMiddle.minus(new Translation2d(2, angleFromMiddle));
                                while (this.getObstacle(newMiddle) != null) {
                                    newMiddle = newMiddle.minus(new Translation2d(0.1, angleFromMiddle));
                                    escapeTimes++;

                                    if (escapeTimes >= 10) break;
                                }
                                if (escapeTimes == 10) continue;
                            }

                            if (!this.bounds.isInOfBounds(newMiddle))
                                continue;

                            middles.add(newMiddle);
                        }

                        middle = middles.stream().sorted(Comparator.comparing(middle::getDistance)).toList().get(0);
                    }

                    trajectory.add(i + 1, new WaypointAutoHeading(middle));
                }
            }
        }

        printStateFinished("Filter " + ", " + trajectory.size() + ", " + getDistributingObstacles(trajectory), started);

        if (this.isFiltering) {
            for (int i = 0; i < trajectory.size() - 1; i++) {
                for (int j = getLatestDefaultWaypointIndex(trajectory, waypoints, i); j > i; j--) {
                    if (!this.isObstacleDistributing(trajectory.get(i).getReferencedPosition(), trajectory.get(j).getReferencedPosition())) {
                        Set<Waypoint> remove = new HashSet<>();
                        for (int k = i + 1; k < j; k++) {
                            remove.add(trajectory.get(k));
                        }
                        trajectory.removeAll(remove);
                        break;
                    }
                }
            }
        }

        if (!this.isPathSafe(trajectory)) {
            return new ArrayList<>(waypoints);
        }

        for (int i = 1; i < trajectory.size() - 2; i++) {
            Waypoint current = trajectory.get(i);
            Waypoint next = trajectory.get(i + 1);
            if (current.getDistance(next) < 1) {
                Waypoint last = trajectory.get(i - 1);
                Waypoint next2 = trajectory.get(i + 2);
                Translation2d merged = current.getReferencedPosition().interpolate(next.getReferencedPosition(), 0.5);
                if (!this.isObstacleDistributing(last.getReferencedPosition(), merged) && !this.isObstacleDistributing(merged, next2.getReferencedPosition())) {
                    trajectory.set(i, new WaypointAutoHeading(merged));
                    trajectory.remove(i + 1);
                }
            }
        }

        printStateFinished("Finished", started);
        return trajectory;
    }

    private int getLatestDefaultWaypointIndex(List<Waypoint> trajectory, List<Waypoint> waypoints, int min) {
        for (int i = min + 1; i <= trajectory.size() - 1; i++) {
            if (waypoints.contains(trajectory.get(i)))
                return i;
        }
        return trajectory.size() - 1;
    }

    private void printStateFinished(String text, long started) {
//        System.out.println(text + ": " + (System.nanoTime() - started) / 1_000_000_000d);
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
            obstacles.addAll(getDistributingObstacle(waypoints.get(i).getReferencedPosition(), waypoints.get(i + 1).getReferencedPosition()));
        }
        return obstacles;
    }

    public boolean isPathSafe(List<Waypoint> waypoints) {
        for (int i = 0; i < waypoints.size() - 1; i++) {
            if (isObstacleDistributing(waypoints.get(i).getReferencedPosition(), waypoints.get(i + 1).getReferencedPosition()))
                return false;
        }
        return true;
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
        return this.extendedObstacles.parallelStream().filter(o -> o.isLineInteracts(initialPose, finalPose)).toList();
    }

    public boolean isObstacleDistributing(Translation2d initialPose, Translation2d finalPose) {
        return this.extendedObstacles.parallelStream().anyMatch(o -> o.isLineInteracts(initialPose, finalPose));
    }

    /**
     * Find an obstacle at the given point
     *
     * @param translation2d the point to find the obstacle
     * @return the obstacle in the given point.
     *         if none found, it will return null.
     */
    public Obstacle getObstacle(Translation2d translation2d) {
        return this.extendedObstacles.parallelStream().filter(o -> o.isPointAt(translation2d)).findFirst().orElse(null);
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

    public void setObstacles(List<Obstacle> obstacles) {
        this.obstacles = obstacles;
        this.extendedObstacles = obstacles.parallelStream().map(o -> o.getExtendedObstacle(this.distanceThreshold)).collect(Collectors.toList());
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
