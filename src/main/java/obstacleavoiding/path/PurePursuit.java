package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.util.Waypoint;

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

    private final Constants constants;

    private final Robot robot;

    private double targetDriveVelocity;
    private double driveVelocity;
    private double omegaVelocity;

    private double lastNormalDriftPercentage;
    private double lastDriftPercentage;

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
        this.robot.drive(new Pose2d());
        this.driveVelocity = 0;
        this.omegaVelocity = 0;
        this.lastUpdate = System.nanoTime();
        this.lastDriftPercentage = 0;

        this.isFinished = false;
    }

    public void update(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel) {
        if (this.isRunning && !this.isFinished) {
            double period = (System.nanoTime() - this.lastUpdate) / 1_000_000_000d;
            boolean isNotLastWaypoint = this.getCurrentWaypointIndex() < this.waypoints.size() - 1;

            Translation2d angle = this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();
            double driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxDriftDistance) / this.constants.maxDriftDistance);
            double slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxSlowDistance) / this.constants.maxSlowDistance);

            driftPercentage = Math.pow(driftPercentage, 1.5);
            slowPercentage = Math.pow(slowPercentage, 2);
            double normalDriftPercentage = driftPercentage;

            Rotation2d driftAngle;
            if (this.getCurrentWaypointIndex() < this.waypoints.size() - 1)
                driftAngle = this.getNextWaypoint().minus(this.getCurrentWaypoint()).getAngle().minus(this.getCurrentWaypoint().minus(this.getPreviousWaypoint()).getAngle());
            else
                driftAngle = Rotation2d.fromDegrees(0);

            double stopVelocity = MathUtil.deadband(this.getDistanceToFinalWaypoint(), 0, this.constants.finalSlowDistance) * this.robot.getConstants().maxVel();
            double maxVelocity = Math.min(this.robot.getConstants().maxVel() - (slowPercentage * (Math.abs(driftAngle.getDegrees()) / 25)), stopVelocity);
            targetDriveVelocity = Math.min(maxVelocity, maxVel);
            driveVelocity += MathUtil.clamp(this.driveController.calculate(this.driveVelocity, this.targetDriveVelocity), -maxAccel * period, maxAccel * period);
            driveVelocity = MathUtil.clamp(driveVelocity, 0, maxVelocity);

            if (isNotLastWaypoint) {
                Translation2d angleFromNext = this.getNextWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();

                if (driftPercentage > 0 && this.lastDriftPercentage > driftPercentage) {
                    driftPercentage = this.lastDriftPercentage + Math.abs(this.lastNormalDriftPercentage - driftPercentage);
                    driftPercentage = Math.min(1, driftPercentage);
                }
                angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }

            double targetOmegaVelocity = this.omegaController.calculate(
                    this.robot.getPosition().getRotation().getDegrees(),
                    (this.getFinalWaypoint().getHeading() - this.getStartWaypoint().getHeading()) * Math.pow((this.getPathDistance() - this.getDistanceToFinalWaypoint()) / this.getPathDistance(), 0.5) + this.getStartWaypoint().getHeading());
            omegaVelocity += MathUtil.clamp(targetOmegaVelocity - omegaVelocity, -maxOmegaAccel * period, maxOmegaAccel * period);
            omegaVelocity = MathUtil.clamp(omegaVelocity, -maxOmegaVel, maxOmegaVel);

            Rotation2d omega = Rotation2d.fromDegrees(omegaVelocity);
            this.robot.drive(new Pose2d(
                    new Translation2d(driveVelocity, angle.getAngle()),
                    omega));

            if (driftPercentage >= 0.9) {
                if (this.getCurrentWaypointIndex() < this.waypoints.size() - 1) {
                    this.currentWaypoint = this.getNextWaypoint();

                    this.lastUpdate = System.nanoTime();
                    this.lastDriftPercentage = 0;
                    this.lastNormalDriftPercentage = 0;
                    return;
                }
            }

            if (this.driveVelocity <= 0.01 && this.getDistanceToFinalWaypoint() <= 0.03 && Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading()) <= 1) {
                this.isFinished = true;
            }

            this.lastUpdate = System.nanoTime();
            this.lastDriftPercentage = driftPercentage;
            this.lastNormalDriftPercentage = normalDriftPercentage;
        }
    }

    public double getDistanceToCurrentWaypoint() {
        return this.robot.getPosition().getTranslation().getDistance(this.getCurrentWaypoint());
    }

    public double getDistanceToFinalWaypoint() {
        int currentWaypoint = this.getCurrentWaypointIndex() + (this.lastDriftPercentage > 0.1 ? 1 : 0);

        double distance = this.robot.getPosition().getTranslation().getDistance(this.getWaypoint(Math.min(currentWaypoint, this.waypoints.size() - 1)));
        for (int i = currentWaypoint; i < this.waypoints.size() - 1; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getPathDistance() {
        double distance = 0;
        for (int i = 0; i < this.waypoints.size() - 1; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getDriftPercentage() {
        return lastDriftPercentage;
    }

    public void setRunning(boolean running) {
        this.isRunning = running;
        if (isRunning) {
            this.lastUpdate = System.nanoTime();
        }
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

    public Waypoint getCurrentWaypoint() {
        return this.currentWaypoint;
    }

    public int getCurrentWaypointIndex() {
        for (int i = 0; i < this.waypoints.size(); i++) {
            if (this.waypoints.get(i).equals(this.currentWaypoint))
                return i;
        }
        this.currentWaypoint = this.getWaypoint(1);
        return 1;
    }

    public Waypoint getPreviousWaypoint() {
        return this.getWaypoint(this.getCurrentWaypointIndex() - 1);
    }

    public Waypoint getNextWaypoint() {
        return this.getWaypoint(this.getCurrentWaypointIndex() + 1);
    }

    public Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }

    public Waypoint getStartWaypoint() {
        return this.getWaypoint(0);
    }

    public Waypoint getFinalWaypoint() {
        return this.getWaypoint(this.waypoints.size() - 1);
    }

    public void setWaypoints(List<Waypoint> waypoints) {
        this.waypoints = waypoints;

    }

    public Constants getConstants() {
        return constants;
    }

    public record Constants(double maxDriftDistance, double maxSlowDistance, double finalSlowDistance) {}
}