package obstacleavoiding.path;

import obstacleavoiding.math.BumbleUtil;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.robot.Robot;
import obstacleavoiding.path.robot.RobotState;
import obstacleavoiding.path.waypoints.NavigationWaypoint;
import obstacleavoiding.path.waypoints.Waypoint;
import obstacleavoiding.path.waypoints.WaypointAutoHeading;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class the controls the movement of the robot in a path of obstacles.
 * It follows a path that was created with some points.
 * He follows it with PID for a certain velocity that calculated by the distance
 * from the final waypoint, the curvature of the path
 */
public class PurePursuit {

    private static final double FF = 0;

    private final ObstacleAvoiding obstacleAvoiding;

    private List<Waypoint> waypoints;

    private final PIDController driveController;
    private final PIDController omegaController;

    private Constants constants;

    private final Robot robot;
    private final double MAX_VELOCITY;

    private Translation2d angle;
    private double targetDriveVelocity;
    private double driveVelocity;
    private double omegaVelocity;

    private double originalDriftPercentage;
    private double driftPercentage;
    private double lastOriginalDriftPercentage;
    private double lastDriftPercentage;

    private double slowPercentage;
    private double anglePercent;

    private Waypoint currentWaypoint;
    private boolean isFinished = false;
    private boolean isRunning = true;

    private double period = 0;

    private Pose2d lastPose = new Pose2d();
    private Pose2d last2Pose = new Pose2d();
    private long lastUpdate;

    public PurePursuit(ObstacleAvoiding obstacleAvoiding, Robot robot, Constants constants, List<Waypoint> waypoints) {
        this.obstacleAvoiding = obstacleAvoiding;
        this.robot = robot;
        this.constants = constants;
        this.waypoints = waypoints;

        MAX_VELOCITY = this.robot.getConstants().maxVel();

        this.driveController = new PIDController(1, 0, 0);
        this.omegaController = new PIDController(3, 0, 0);
        this.omegaController.enableContinuousInput(0, 2 * Math.PI);
    }

    public PurePursuit(ObstacleAvoiding obstacleAvoiding, Robot robot, Constants constants, Waypoint... waypoints) {
        this(obstacleAvoiding, robot, constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public void reset() {
        this.currentWaypoint = this.waypoints.get(1);
        this.driveVelocity = this.robot.getVelocity().getTranslation().getNorm();
        this.omegaVelocity = this.robot.getVelocity().getRotation().getDegrees();
        this.lastUpdate = System.nanoTime();
        this.lastDriftPercentage = 0;
        this.slowPercentage = 0;

        this.isFinished = false;
    }

    public void update(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel) {
        this.update(maxVel, maxAccel, maxOmegaVel, maxOmegaAccel, (System.nanoTime() - this.lastUpdate) / 1_000_000_000d);
    }

    public void update(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period) {
        this.period = period;

        if (!this.isRunning || this.isFinished)
            return;

        int currentIndex = this.getCurrentWaypointIndex();
        boolean isNotLastWaypoint = currentIndex < this.waypoints.size() - 1;

        this.lastDriftPercentage = driftPercentage;
        this.lastOriginalDriftPercentage = originalDriftPercentage;

        angle = normalize(this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()));

        double maxDriftDistance = this.constants.maxDriftDistance;
        maxDriftDistance *= MathUtil.clamp(getPreviousWaypoint().getDistance(getCurrentWaypoint()) / (1.5 * this.constants.maxDriftDistance), 0, 1);
        maxDriftDistance = Math.max(maxDriftDistance, 0.5);

        driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / maxDriftDistance, 0, 1));
        driftPercentage = Math.pow(driftPercentage, this.constants.driftPercentLinearity);
        originalDriftPercentage = driftPercentage;

        if (isNotLastWaypoint) {
            Translation2d angleFromNext = normalize(this.getNextWaypoint().minus(this.robot.getPosition().getTranslation()));

            if (driftPercentage > 0 && this.lastDriftPercentage > driftPercentage) {
                driftPercentage = this.lastDriftPercentage + Math.abs(this.lastOriginalDriftPercentage - driftPercentage);
                driftPercentage = Math.min(driftPercentage, 1);
            }

            if (!(this.currentWaypoint instanceof NavigationWaypoint)) {
                angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }
        }

        double driftVelocity = maxVel;
        if (isNotLastWaypoint) {
            double turnAngle = this.getNextWaypoint().minus(this.getCurrentWaypoint()).getAngle().minus(this.getCurrentWaypoint().minus(this.getPreviousWaypoint()).getAngle()).getDegrees();
            turnAngle = Math.abs(BumbleUtil.bound180Degrees(turnAngle));

            double maxSlowDistance = (Math.pow(this.calculateDriftVelocity(turnAngle, 1), 2) - Math.pow(MAX_VELOCITY, 2)) / (2 * -this.constants.deceleration);
            slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / maxSlowDistance, 0, 1));
            slowPercentage = Math.pow(slowPercentage, this.constants.slowPercentLinearity);

            driftVelocity = this.calculateDriftVelocity(turnAngle, slowPercentage);
        }

        double finalSlowDistance = Math.pow(MAX_VELOCITY, 2) / (2 * this.constants.deceleration);
        double stopVelocity = MathUtil.clamp(this.getDistanceToFinalWaypoint() / finalSlowDistance, 0, 1);
        stopVelocity = Math.pow(stopVelocity, 1 / this.constants.finalSlowPercentLinearity);
        stopVelocity *= MAX_VELOCITY;

        targetDriveVelocity = Math.min(Math.min(driftVelocity, stopVelocity), maxVel);

        driveVelocity += MathUtil.clamp(this.driveController.calculate(this.getCurrentDrivetrainVelocity(), this.targetDriveVelocity), -maxAccel * period, maxAccel * period);
        driveVelocity = MathUtil.clamp(driveVelocity, 0, this.targetDriveVelocity);

        int lastHeadingWaypoint = this.getLastHeadingWaypointIndex();
        int nextHeadingWaypoint = this.getNextHeadingWaypointIndex();

        // double absoluteDistance = this.getDistance(lastHeadingWaypoint, nextHeadingWaypoint);
        // anglePercent = 1 - (this.getDistance(nextHeadingWaypoint) / absoluteDistance);
        // if (anglePercent >= 0.01) {
        //     anglePercent = Math.pow(anglePercent, this.constants.rotationPercentLinearity);
        // }
        anglePercent = 1;

        double targetAngle = this.waypoints.get(lastHeadingWaypoint).getHeading().getRadians() +
                (anglePercent * BumbleUtil.boundPIRadians(this.waypoints.get(nextHeadingWaypoint).getHeading().minus(this.waypoints.get(lastHeadingWaypoint).getHeading()).getRadians()));
        targetAngle = BumbleUtil.boundPIRadians(targetAngle);

        double targetOmegaVelocity = this.omegaController.calculate(BumbleUtil.boundPIRadians(this.robot.getPosition().getRotation().getRadians()), targetAngle);
        omegaVelocity += MathUtil.clamp(targetOmegaVelocity - omegaVelocity, -maxOmegaAccel * period, maxOmegaAccel * period);
        omegaVelocity = MathUtil.clamp(omegaVelocity, -maxOmegaVel, maxOmegaVel);

        this.last2Pose = this.lastPose;
        this.lastPose = this.robot.getPosition();

        double velocity = driveVelocity + FF;
        this.robot.drive(new Pose2d(
                new Translation2d(velocity, angle.getAngle()),
                Rotation2d.fromRadians(omegaVelocity)), period, false);

        if (currentIndex < this.waypoints.size() - 1 &&
                (driftPercentage >= 0.9 || this.isAbleToContinueNextWaypoint())) {
            this.currentWaypoint = this.waypoints.get(currentIndex + 1);
            this.lastDriftPercentage = 0;
            this.driftPercentage = 0;
            this.slowPercentage = 0;
        }

        this.lastUpdate = System.nanoTime();

        if (this.driveVelocity <= this.constants.velocityTolerance && this.getDistanceToFinalWaypoint() <= this.constants.distanceTolerance &&
                Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading().getDegrees()) <= this.constants.rotationPercentLinearity) {
            this.isFinished = true;
        }
    }

    private double calculateDriftVelocity(double angle, double slowPercentage) {
        double driftVelocity = angle * slowPercentage;
        driftVelocity = Math.pow((180 - driftVelocity) / 180, this.constants.turnVelocityLinearity);
        driftVelocity *= MAX_VELOCITY - this.constants.minimumDriftVelocity;
        driftVelocity += this.constants.minimumDriftVelocity;
        return driftVelocity;
    }

    private boolean isAbleToContinueNextWaypoint() {
        return !(this.getWaypoint(this.getCurrentWaypointIndex()) instanceof NavigationWaypoint) &&
                !this.obstacleAvoiding.isObstacleDistributing(this.robot.getPosition().getTranslation(), this.getNextWaypoint()) &&
                BumbleUtil.bound360Degrees(angle.getAngle().minus(this.robot.getPosition().getTranslation().minus(this.getCurrentWaypoint()).getAngle()).getDegrees()) <= 10;
    }

    public List<RobotState> getEstimatedPath(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period) {
        List<RobotState> poses = new ArrayList<>();
        Robot robot = new Robot(this.getStartWaypoint().getPose2d(), this.robot.getConstants());
        PurePursuit purePursuit = new PurePursuit(this.obstacleAvoiding, robot, this.constants, new ArrayList<>(this.waypoints));
        purePursuit.reset();
        while (!purePursuit.isFinished()) {
            purePursuit.update(maxVel, maxAccel, maxOmegaVel, maxOmegaAccel, period);
            poses.add(new RobotState(robot.getPosition(), robot.getVelocity().getTranslation().getNorm(),
                    MathUtil.limitDot(purePursuit.lastDriftPercentage, 3), MathUtil.limitDot(purePursuit.slowPercentage, 3),
                    MathUtil.limitDot(purePursuit.getDistanceToCurrentWaypoint(), 3)));
        }
        return poses;
    }

    public double getCurrentDrivetrainVelocity() {
        return this.robot.getVelocity().getTranslation().getNorm();
    }

    public int getNextHeadingWaypointIndex() {
        for (int i = this.getCurrentWaypointIndex(); i < this.waypoints.size(); i++) {
            if (!(this.waypoints.get(i) instanceof WaypointAutoHeading))
                return i;
        }
        return -1;
    }

    public int getLastHeadingWaypointIndex() {
        int current = this.getCurrentWaypointIndex();
        for (int i = current - 1; i >= 0; i--) {
            if (!(this.waypoints.get(i) instanceof WaypointAutoHeading))
                return i;
        }
        return -1;
    }

    private Translation2d normalize(Translation2d translation2d) {
        return translation2d.div(translation2d.getNorm() == 0 ? 1 : translation2d.getNorm());
    }

    public double getCurvatureRadius() {
        Translation2d current = this.robot.getPosition().getTranslation();
        Translation2d last = current.minus(new Translation2d(-1, lastPose.getTranslation().minus(current).getAngle()));
        Translation2d last2 = last.minus(new Translation2d(-1, last2Pose.getTranslation().minus(last).getAngle()));

        double d1x = this.calculateDerivative(current.getX(), last.getX());
        double d1y = this.calculateDerivative(current.getY(), last.getY());
        double d2x = this.calculateDerivative(d1x, calculateDerivative(last.getX(), last2.getX()));
        double d2y = this.calculateDerivative(d1y, calculateDerivative(last.getY(), last2.getY()));

        return Math.pow((d1x * d1x) + (d1y * d1y), 1.5) / ((d1x * d2y) - (d1y * d2x));
    }

    public double calculateDerivative(double current, double last) {
        return (current - last) / period;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(this.robot.getPosition().getX() - this.lastPose.getX(), this.robot.getPosition().getY() - this.lastPose.getY());
    }

    public double getDistance(int waypoint1, int waypoint2) {
        double distance = 0;
        for (int i = waypoint1; i < waypoint2; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getDistance(int waypoint) {
        int currentWaypoint = this.getCurrentWaypointIndex() + (this.lastDriftPercentage > 0.1 ? 1 : 0);
        return this.getDistance(currentWaypoint, waypoint) +
                this.robot.getPosition().getTranslation().getDistance(this.getWaypointPosition(Math.min(currentWaypoint, this.waypoints.size() - 1)));
    }

    public double getDistanceToCurrentWaypoint() {
        return this.robot.getPosition().getTranslation().getDistance(this.getCurrentWaypoint());
    }

    public double getDistanceToFinalWaypoint() {
        return this.getDistance(this.waypoints.size() - 1);
    }

    public double getPathDistance() {
        return this.getDistance(0, this.waypoints.size() - 1);
    }

    public double getDriftPercentage() {
        return driftPercentage;
    }

    public double getSlowPercentage() {
        return slowPercentage;
    }

    public double getAnglePercent() {
        return anglePercent;
    }

    public void setRunning(boolean running) {
        if (!this.isRunning && running) {
            this.lastUpdate = System.nanoTime();
        }
        this.isRunning = running;
    }

    public boolean isRunning() {
        return isRunning && !this.isFinished;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public double getTargetDriveVelocity() {
        return targetDriveVelocity;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public Translation2d getCurrentWaypoint() {
        return this.currentWaypoint.getReferencedPosition();
    }

    public int getCurrentWaypointIndex() {
        for (int i = 1; i < this.waypoints.size(); i++) {
            if (this.waypoints.get(i).equals(this.currentWaypoint))
                return i;
        }
        this.currentWaypoint = this.waypoints.get(1);
        return 1;
    }

    public Translation2d getPreviousWaypoint() {
        return this.getWaypointPosition(this.getCurrentWaypointIndex() - 1);
    }

    public Translation2d getNextWaypoint() {
        return this.getWaypointPosition(this.getCurrentWaypointIndex() + 1);
    }

    public Translation2d getWaypointPosition(int index) {
        return waypoints.get(index).getReferencedPosition();
    }

    public Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }

    public Waypoint getStartWaypoint() {
        return this.waypoints.get(0);
    }

    public Waypoint getFinalWaypoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public void setWaypoints(List<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public Constants getConstants() {
        return constants;
    }

    public void setConstants(Constants constants) {
        this.constants = constants;
    }

    public record Constants(double maxDriftDistance, double driftPercentLinearity,
                            double turnVelocityLinearity, double slowPercentLinearity,
                            double finalSlowPercentLinearity, double rotationPercentLinearity,
                            double deceleration, double minimumDriftVelocity,
                            double distanceTolerance, double velocityTolerance, double rotationTolerance) {}
}
