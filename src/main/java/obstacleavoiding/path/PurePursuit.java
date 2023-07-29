package obstacleavoiding.path;

import obstacleavoiding.math.BumbleUtil;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.math.util.KalmanFilter;
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

    private double targetDriveVelocity;
    private double driveVelocity;
    private double omegaVelocity;

    private Translation2d angle;
    private double lastNormalDriftPercentage;
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

        this.isFinished = false;
    }

    public void update(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel) {
        this.update(maxVel, maxAccel, maxOmegaVel, maxOmegaAccel, (System.nanoTime() - this.lastUpdate) / 1_000_000_000d);
    }

    public void update(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period) {
        this.period = period;
        double MAX_VELOCITY = this.robot.getConstants().maxVel();
        if (this.isRunning && !this.isFinished) {
            boolean isNotLastWaypoint = this.getCurrentWaypointIndex() < this.waypoints.size() - 1;

            // The angle from the robot to the current waypoint.
            angle = normalize(this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()));

            double maxDriftDistance = this.constants.maxDriftDistance *
                    MathUtil.clamp(getPreviousWaypoint().getDistance(getCurrentWaypoint()) / (2 * this.constants.maxDriftDistance), 0, 1);
            maxDriftDistance = Math.max(maxDriftDistance, 0.5);

            double driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / maxDriftDistance, 0, 1));
            driftPercentage = Math.pow(driftPercentage, this.constants.driftPercentLinearity);
            double normalDriftPercentage = driftPercentage;

            // Calculating the needed drive angle at turn with the driftPercent.
            if (isNotLastWaypoint) {
                // Calculating the angle from the robot to the next waypoint.
                Translation2d angleFromNext = normalize(this.getNextWaypoint().minus(this.robot.getPosition().getTranslation()));

                // Changing the driftPercent when getting further from the current waypoint to change the current waypoint to the next.
                if (driftPercentage > 0 && this.lastDriftPercentage > driftPercentage) {
                    driftPercentage = this.lastDriftPercentage + Math.abs(this.lastNormalDriftPercentage - driftPercentage);
                    driftPercentage = Math.min(driftPercentage, 1);
                }

                // Calculating the angle with driftPercentage by interpolating the angle from the robot to the current waypoint and to the next waypoint.
                if (!(this.currentWaypoint instanceof NavigationWaypoint))
                    angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }

            slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / this.constants.maxSlowDistance, 0, 1));
            slowPercentage = Math.pow(slowPercentage, this.constants.slowPercentLinearity);
//            slowPercentage = Math.sin(slowPercentage * Math.PI);

            // Checking if the target waypoint is NavigationWaypoint to aim for the target velocity of the waypoint.
            if (this.currentWaypoint instanceof NavigationWaypoint && ((NavigationWaypoint) this.currentWaypoint).getTargetVelocity() < maxVel) {
                NavigationWaypoint navigation = (NavigationWaypoint) this.currentWaypoint;
                // Changed the maxVel to interpolation of the given maxVel and target max vel of the waypoint.
                maxVel = (maxVel - navigation.getTargetVelocity()) * (1 - slowPercentage) + navigation.getTargetVelocity();
            }

            // Calculating the drift level of the robot based on its current, previous, and next waypoints.
            // driftVelocity is the amount to slow down at a turn.
            double driftVelocity = maxVel;
            if (isNotLastWaypoint) {
                driftVelocity = this.getNextWaypoint().minus(this.getCurrentWaypoint()).getAngle().minus(this.getCurrentWaypoint().minus(this.getPreviousWaypoint()).getAngle()).getDegrees();
                driftVelocity = Math.abs(BumbleUtil.bound180Degrees(driftVelocity));

                driftVelocity *= slowPercentage;
                driftVelocity = Math.pow((180 - driftVelocity) / 180, 2);
                driftVelocity *= this.robot.getConstants().maxVel() - this.constants.minimumDriftVelocity;
                driftVelocity += this.constants.minimumDriftVelocity;
            }

            double finalSlowDistance = Math.pow(MAX_VELOCITY, 2) / (2 * this.constants.deceleration);
            // double finalSlowDistance = this.constants.maxFinalSlowDistance;
            // double stopVelocity = MAX_VELOCITY * MathUtil.clamp(this.getDistanceToFinalWaypoint() / finalSlowDistance, 0, 1);
            double stopVelocity = MathUtil.clamp(this.getDistanceToFinalWaypoint() / finalSlowDistance, 0, 1);
            stopVelocity = Math.pow(stopVelocity, 1 / this.constants.finalSlowPercentLinearity);
            stopVelocity *= MAX_VELOCITY;

            // Choosing the lowest velocity for being able to stop, turn and not going over the maxVel.
            targetDriveVelocity = Math.min(Math.min(driftVelocity, stopVelocity), maxVel);

            // Calculating the velocity of the robot with acceleration limits
            driveVelocity += MathUtil.clamp(this.driveController.calculate(this.getCurrentDrivetrainVelocity(), this.targetDriveVelocity), -maxAccel * period, maxAccel * period);
            driveVelocity = MathUtil.clamp(driveVelocity, 0, this.targetDriveVelocity);

            int lastHeadingWaypoint = this.getLastHeadingWaypointIndex();
            int nextHeadingWaypoint = this.getNextHeadingWaypointIndex();

            // Calculating the percentage of the path.
            // double absoluteDistance = this.getDistance(lastHeadingWaypoint, nextHeadingWaypoint);
            // anglePercent = 1 - (this.getDistance(nextHeadingWaypoint) / absoluteDistance);
            // if (anglePercent >= 0.01) {
            // anglePercent = Math.pow(anglePercent, this.constants.rotationPercentLinearity);
            // }
            anglePercent = 1;

            // Calculating the target angle of the robot at the current path position by interpolating the last and next heading waypoint.
            double targetAngle = this.waypoints.get(lastHeadingWaypoint).getHeading().getRadians() +
                    (anglePercent * BumbleUtil.boundPIRadians(this.waypoints.get(nextHeadingWaypoint).getHeading().minus(this.waypoints.get(lastHeadingWaypoint).getHeading()).getRadians()));
            targetAngle = BumbleUtil.boundPIRadians(targetAngle);

            // Calculating the omega velocity with normal pid.
            double targetOmegaVelocity = this.omegaController.calculate(BumbleUtil.boundPIRadians(this.robot.getPosition().getRotation().getRadians()), targetAngle);
            // Limiting the velocity with the given acceleration and velocity.
            omegaVelocity += MathUtil.clamp(BumbleUtil.boundPIRadians(targetOmegaVelocity - omegaVelocity), -maxOmegaAccel * period, maxOmegaAccel * period);
            omegaVelocity = MathUtil.clamp(BumbleUtil.boundPIRadians(omegaVelocity), -maxOmegaVel, maxOmegaVel);

            this.last2Pose = this.lastPose;
            this.lastPose = this.robot.getPosition();

            double velocity = driveVelocity + FF;
            // Sends the velocity to the robot.
            this.robot.drive(new Pose2d(
                    new Translation2d(velocity, angle.getAngle()),
                    Rotation2d.fromRadians(omegaVelocity)), period, false);

            // Checking if the drift percentage is close to the end to change to the next waypoint.
            if (this.getCurrentWaypointIndex() < this.waypoints.size() - 1 &&
                    (driftPercentage >= 0.9 || this.isAbleToContinueNextWaypoint())) {
                this.currentWaypoint = this.waypoints.get(this.getCurrentWaypointIndex() + 1);

                this.lastUpdate = System.nanoTime();
                this.lastDriftPercentage = 0;
                this.lastNormalDriftPercentage = 0;
                return;
            }

            // Update values for the next run.
            this.lastUpdate = System.nanoTime();
            this.lastDriftPercentage = driftPercentage;
            this.lastNormalDriftPercentage = normalDriftPercentage;

            if (this.driveVelocity <= this.constants.velocityTolerance && this.getDistanceToFinalWaypoint() <= this.constants.distanceTolerance &&
                    Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading().getDegrees()) <= this.constants.rotationPercentLinearity) {
                this.isFinished = true;
            }
        }
    }

    private boolean isAbleToContinueNextWaypoint() {
        return !(this.getWaypoint(this.getCurrentWaypointIndex()) instanceof NavigationWaypoint) &&
                !this.obstacleAvoiding.isObstacleDistributing(this.robot.getPosition().getTranslation(), this.getNextWaypoint()) &&
                BumbleUtil.bound360Degrees(angle.getAngle().minus(this.robot.getPosition().getTranslation().minus(this.getCurrentWaypoint()).getAngle()).getDegrees()) <= 20;
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
                    MathUtil.limitDot(purePursuit.getCurvatureRadius(), 3)));
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
        double d1x = this.calculateDerivative(this.robot.getPosition().getX(), this.lastPose.getX());
        double d1y = this.calculateDerivative(this.robot.getPosition().getY(), this.lastPose.getY());
        double d2x = this.calculateDerivative(d1x, calculateDerivative(this.lastPose.getX(), this.last2Pose.getX()));
        double d2y = this.calculateDerivative(d1y, calculateDerivative(this.lastPose.getY(), this.last2Pose.getY()));

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
        return lastDriftPercentage;
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

    public record Constants(double maxDriftDistance, double maxSlowDistance,
                            double driftPercentLinearity, double slowPercentLinearity,
                            double finalSlowPercentLinearity, double rotationPercentLinearity,
                            double deceleration, double minimumDriftVelocity,
                            double distanceTolerance, double velocityTolerance, double rotationTolerance) {}
}
