package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

public abstract class Path {
    protected final double dx;

    protected List<Waypoint> waypoints;
    protected final double differentBetweenTs;

    protected final Constants constants;

    public Path(Constants constants, double dx, List<Waypoint> waypoints) {
        this.constants = constants;
        this.dx = dx;
        this.waypoints = waypoints;
        this.differentBetweenTs = 0.01 / waypoints.size();
    }

    public Path(Constants constants, double dx, Waypoint... waypoints) {
        this(constants, dx, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public State getClosestPoint(Pose2d robotPose) {
        double minT = 0;
        double minDistance = Double.MAX_VALUE;
        Pose2d output = this.getPosition(0);
        for (double t = this.differentBetweenTs; t <= 1; t += this.differentBetweenTs) {
            Pose2d pose = this.getPosition(t);
            double distance = robotPose.getTranslation().getDistance(pose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                output = pose;
                minT = t;
            }
        }

        return new State(output, minT);
    }

    public Pose2d getVelocity(State state, Robot robot, double velocity, double omega) {
        Translation2d vector = new Translation2d(1 - constants.errorCorrectorPower(), this.getAngle(state.t()))
                .plus(state.pose().getTranslation().minus(robot.getPosition().getTranslation()).times(constants.errorCorrectorPower()));

        double curvature = 1 / Math.abs(this.getCurvatureRadius(state.t()));
        velocity = Math.min(constants.maxVel() - Math.min(curvature, 3.5), velocity);

        return new Pose2d(new Translation2d(velocity, vector.getAngle()), Rotation2d.fromDegrees(omega));
    }

    public abstract double getX(double t);

    public abstract double getY(double t);

    public Translation2d getLocation(double t) {
        return new Translation2d(getX(t), getY(t));
    }

    public Rotation2d getAngle(double t) {
        Translation2d angle = this.getLocation(t + this.dx).minus(this.getLocation(t - this.dx));
        return new Rotation2d(angle.getX(), angle.getY());
    }

    public Pose2d getPosition(double t) {
        return new Pose2d(this.getLocation(t), this.getAngle(t));
    }

    public double getXDerivative(double t) {
        return MathUtil.calculateDerivative(t, this::getX);
    }

    public double getYDerivative(double t) {
        return MathUtil.calculateDerivative(t, this::getY);
    }

    public double getXSecondDerivative(double t) {
        return MathUtil.calculateDerivative(t, this::getXDerivative);
    }

    public double getYSecondDerivative(double t) {
        return MathUtil.calculateDerivative(t, this::getYDerivative);
    }

    public double getDistance(double t1, double t2) {
        return MathUtil.calculateIntegral(t1, t2, t -> Math.hypot(this.getXDerivative(t), this.getYDerivative(t)));
    }

    public double getDistance(double t) {
        return this.getDistance(0, t);
    }


    public double getCurvatureRadius(double t) {
        double d1x = this.getXDerivative(t);
        double d1y = this.getYDerivative(t);
        double d2x = this.getXSecondDerivative(t);
        double d2y = this.getYSecondDerivative(t);

        return Math.pow((d1x * d1x) + (d1y * d1y), 1.5) / ((d1x * d2y) - (d1y * d2x));
    }

    public Translation2d getFinalPoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public Translation2d getStartPoint() {
        return this.waypoints.get(0);
    }

    public List<Waypoint> getWaypoints() {
        return this.waypoints;
    }

    public Waypoint getWaypoint(int index) {
        return this.waypoints.get(index);
    }

    public void setWaypoint(int index, Waypoint waypoint) {
        this.waypoints.set(index, waypoint);
    }

    public double getDifferentBetweenTs() {
        return this.differentBetweenTs;
    }

    public double getPathLength() {
        return this.getDistance(1);
    }

    public Constants getConstants() {
        return this.constants;
    }

    public void setWaypoints(List<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public record State(Pose2d pose, double t) {}


    public record Constants(double maxVel, double maxAccel, double errorCorrectorPower) {}

}
