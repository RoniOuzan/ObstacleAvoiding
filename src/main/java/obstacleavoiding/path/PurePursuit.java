package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.robot.Robot;
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
    private List<Waypoint> waypoints;

    private final PIDController driveController;
    private final PIDController omegaController;

    private Constants constants;

    private final Robot robot;

    private double targetDriveVelocity;
    private double driveVelocity;
    private double omegaVelocity;

    private double lastNormalDriftPercentage;
    private double lastDriftPercentage;
    private double slowPercentage;

    private Waypoint currentWaypoint;
    private boolean isFinished = false;
    private boolean isRunning = true;

    private long lastUpdate;

    public PurePursuit(Robot robot, Constants constants, List<Waypoint> waypoints) {
        this.robot = robot;
        this.constants = constants;
        this.waypoints = waypoints;

        this.driveController = new PIDController(0.3, 0, 0);
        this.omegaController = new PIDController(3, 0, 0);
        this.omegaController.enableContinuousInput(-180, 180);
    }

    public PurePursuit(Robot robot, Constants constants, Waypoint... waypoints) {
        this(robot, constants, new ArrayList<>(Arrays.asList(waypoints)));
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
        if (this.isRunning && !this.isFinished) {
            boolean isNotLastWaypoint = this.getCurrentWaypointIndex() < this.waypoints.size() - 1;

            // The angle from the robot to the current waypoint.
            Translation2d angle = this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();

            // Calculating the drift percentage and slow percentage based on the distance to the current waypoint.
            double driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / this.constants.maxDriftDistance, 0, 1));
            slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / this.constants.maxSlowDistance, 0, 1));

            // Calculating the exponential value of the values. The exponent value is determined by the given constants.
            driftPercentage = Math.pow(driftPercentage, this.constants.driftPercentLinearity);
            slowPercentage = Math.pow(slowPercentage, this.constants.slowPercentLinearity);
            double normalDriftPercentage = driftPercentage;

            // Calculating the drift level of the robot based on its current, previous, and next waypoints.
            // driftLevel is the amount to slow down at a turn.
            double driftLevel = 0;
            if (isNotLastWaypoint) {
                driftLevel = this.getNextWaypoint().minus(this.getCurrentWaypoint()).getAngle().minus(this.getCurrentWaypoint().minus(this.getPreviousWaypoint()).getAngle()).getDegrees();
                driftLevel = Math.abs(MathUtil.inputModulus(driftLevel, -180, 180)) / this.constants.driftAngleDivider;
            }

            // Checking if the target waypoint is NavigationWaypoint to aim for the target velocity of the waypoint.
            if (this.currentWaypoint instanceof NavigationWaypoint navigation && navigation.getTargetVelocity() < maxVel) {
                // Changed the maxVel to interpolation of the given maxVel and target max vel of the waypoint.
                maxVel = (maxVel - navigation.getTargetVelocity()) * (1 - slowPercentage) + navigation.getTargetVelocity();
            }

            // Calculating the needed velocity to stop the robot exactly at the target.
            double stopVelocity = 1 - MathUtil.clamp(this.getDistanceToFinalWaypoint() / this.constants.finalSlowDistance, 0, 1);
            stopVelocity = this.robot.getConstants().maxVel() * (1 - Math.pow(stopVelocity, this.constants.finalSlowPercentLinearity));

            // Calculating the drift velocity to make the turn as fast as possible with the given constants.
            double maxVelocity = this.robot.getConstants().maxVel() - Math.min(slowPercentage * driftLevel, this.robot.getConstants().maxVel() - this.constants.minimumDriftVelocity);

            // Choosing the lowest velocity for being able to stop, turn and not going over the maxVel.
            targetDriveVelocity = Math.min(Math.min(maxVelocity, stopVelocity), maxVel);

            // Calculating the velocity of the robot with acceleration limits
            driveVelocity += MathUtil.clamp(this.driveController.calculate(this.robot.getVelocity().getTranslation().getNorm(), this.targetDriveVelocity), -maxAccel * period, maxAccel * period);
            driveVelocity = MathUtil.clamp(driveVelocity, 0, targetDriveVelocity);

            // Calculating the needed drive angle at turn with the driftPercent.
            if (isNotLastWaypoint) {
                // Calculating the angle from the robot to the next waypoint.
                Translation2d angleFromNext = this.getNextWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();

                // Changing the driftPercent when getting further from the current waypoint to change the current waypoint to the next.
                if (driftPercentage > 0 && this.lastDriftPercentage > driftPercentage) {
                    driftPercentage = this.lastDriftPercentage + Math.abs(this.lastNormalDriftPercentage - driftPercentage);
                    driftPercentage = Math.min(driftPercentage, 1);
                }

                // Calculating the angle with driftPercentage by interpolating the angle from the robot to the current waypoint and to the next waypoint.
                if (!(this.currentWaypoint instanceof NavigationWaypoint))
                    angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }

            int lastHeading = this.getLastHeadingWaypointIndex();
            int nextHeading = this.getNextHeadingWaypointIndex();

            // Calculating the percentage of the path.
            double absoluteDistance = this.getDistance(lastHeading, nextHeading);
            double anglePercent = (absoluteDistance - this.getDistance(nextHeading)) / absoluteDistance;
            if (anglePercent >= 0.001)
                anglePercent = Math.pow(anglePercent, this.constants.rotationPercentLinearity);

            // Calculating the target angle of the robot at the current path position by interpolating the last and next heading waypoint.
            double targetAngle = this.waypoints.get(lastHeading).getHeading() +
                    ((this.waypoints.get(nextHeading).getHeading() - this.waypoints.get(lastHeading).getHeading()) * anglePercent);

            // Calculating the omega velocity with normal pid.
            double targetOmegaVelocity = this.omegaController.calculate(this.robot.getPosition().getRotation().getDegrees(), targetAngle);
            // Limiting the velocity with the given acceleration and velocity.
            omegaVelocity += MathUtil.clamp(targetOmegaVelocity - omegaVelocity, -maxOmegaAccel * period, maxOmegaAccel * period);
            omegaVelocity = MathUtil.clamp(omegaVelocity, -maxOmegaVel, maxOmegaVel);

            // Sends the velocity to the robot.
            this.robot.drive(new Pose2d(
                    new Translation2d(driveVelocity, angle.getAngle()),
                    Rotation2d.fromDegrees(omegaVelocity)), period, false);

            // Checking if the drift percentage is close to the end to change to the next waypoint.
            if (driftPercentage >= 0.95) {
                if (this.getCurrentWaypointIndex() < this.waypoints.size() - 1) {
                    this.currentWaypoint = this.waypoints.get(this.getCurrentWaypointIndex() + 1);

                    this.lastUpdate = System.nanoTime();
                    this.lastDriftPercentage = 0;
                    this.lastNormalDriftPercentage = 0;
                    return;
                }
            }

            if (this.driveVelocity <= this.constants.velocityTolerance && this.getDistanceToFinalWaypoint() <= this.constants.distanceTolerance &&
                    Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading()) <= this.constants.rotationPercentLinearity) {
                this.isFinished = true;
            }

            // Update values for the next run.
            this.lastUpdate = System.nanoTime();
            this.lastDriftPercentage = driftPercentage;
            this.lastNormalDriftPercentage = normalDriftPercentage;
        }
    }

    public List<Pose2d> getEstimatedPath(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period) {
        List<Pose2d> poses = new ArrayList<>();
        Robot robot = new Robot(this.getStartWaypoint().getPose2d(), this.robot.getConstants());
        PurePursuit purePursuit = new PurePursuit(robot, this.constants, new ArrayList<>(this.waypoints));
        purePursuit.reset();
        while (!purePursuit.isFinished()) {
            purePursuit.update(maxVel, maxAccel, maxOmegaVel, maxOmegaAccel, period);
            poses.add(robot.getPosition());
        }
        return poses;
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

    public void setLinearConstants(double driftPercentLinearity, double slowPercentLinearity, double finalSlowPercentLinearity, double rotationPercentLinearity) {
        this.constants = new Constants(this.constants.maxDriftDistance, this.constants.maxSlowDistance, this.constants.finalSlowDistance,
                driftPercentLinearity, slowPercentLinearity, finalSlowPercentLinearity, rotationPercentLinearity,
                this.constants.driftAngleDivider, this.constants.minimumDriftVelocity,
                this.constants.distanceTolerance, this.constants.velocityTolerance, this.constants.rotationTolerance);
    }

    public void setDistanceConstants(double maxDriftDistance, double maxSlowDistance, double finalSlowDistance) {
        this.constants = new Constants(maxDriftDistance, maxSlowDistance, finalSlowDistance,
                this.constants.driftPercentLinearity, this.constants.slowPercentLinearity, this.constants.finalSlowPercentLinearity, this.constants.rotationPercentLinearity,
                this.constants.driftAngleDivider, this.constants.minimumDriftVelocity,
                this.constants.distanceTolerance, this.constants.velocityTolerance, this.constants.rotationTolerance);
    }

    public void setDriftVelocityConstants(double driftAngleDivider, double minimumDriftVelocity) {
        this.constants = new Constants(this.constants.maxDriftDistance, this.constants.maxSlowDistance, this.constants.finalSlowDistance,
                this.constants.driftPercentLinearity, this.constants.slowPercentLinearity, this.constants.finalSlowPercentLinearity, this.constants.rotationPercentLinearity,
                driftAngleDivider, minimumDriftVelocity,
                this.constants.distanceTolerance, this.constants.velocityTolerance, this.constants.rotationTolerance);
    }

    public record Constants(double maxDriftDistance, double maxSlowDistance, double finalSlowDistance,
                            double driftPercentLinearity, double slowPercentLinearity, double finalSlowPercentLinearity, double rotationPercentLinearity,
                            double driftAngleDivider, double minimumDriftVelocity,
                            double distanceTolerance, double velocityTolerance, double rotationTolerance) {}
}
