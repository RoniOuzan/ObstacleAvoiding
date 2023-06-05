package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.pid.PIDPreset;
import obstacleavoiding.path.pid.ProfiledPIDController;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
        this.robot.setPosition(new Pose2d(this.getStartWaypoint(), Rotation2d.fromDegrees(this.getStartWaypoint().getHeading())));
        this.robot.drive(new Pose2d());
        this.driveVelocity = 0;
        this.omegaVelocity = 0;
        this.lastUpdate = System.nanoTime();
        this.lastDriftPercentage = 0;

        this.isFinished = false;
    }

    public void update() {
        if (this.isRunning && !this.isFinished) {
            double period = (System.nanoTime() - this.lastUpdate) / 1_000_000_000d;
            boolean isNotLastWaypoint = this.getCurrentWaypointIndex() < this.waypoints.size() - 1;

            Translation2d angle = this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();
            double driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxDriftDistance) / this.constants.maxDriftDistance);
            double slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxSlowDistance) / this.constants.maxSlowDistance);

            driftPercentage = Math.pow(driftPercentage, 1.5);
            slowPercentage = Math.pow(slowPercentage, 3);
            double normalDriftPercentage = driftPercentage;

            Rotation2d driftAngle;
            if (this.getCurrentWaypointIndex() < this.waypoints.size() - 1)
                driftAngle = this.getNextWaypoint().minus(this.getCurrentWaypoint()).getAngle().minus(this.getCurrentWaypoint().minus(this.getPreviousWaypoint()).getAngle());
            else
                driftAngle = Rotation2d.fromDegrees(0);

            double closeSlower = (1 - MathUtil.deadband(this.getDistanceToFinalWaypoint(), 0, this.constants.finalSlowDistance)) * this.constants.maxVel;
            double maxVelocity = this.constants.maxVel - (slowPercentage * Math.abs(driftAngle.getDegrees()) / 30) - closeSlower;
            targetDriveVelocity = maxVelocity;
            driveVelocity += MathUtil.clamp(this.driveController.calculate(this.driveVelocity, this.targetDriveVelocity), -this.constants.maxAccel * period, this.constants.maxAccel * period);
            driveVelocity = MathUtil.clamp(driveVelocity, 0, maxVelocity);

            if (isNotLastWaypoint) {
                Translation2d angleFromNext = this.getNextWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();

                if (driftPercentage > 0 && this.lastDriftPercentage > driftPercentage) {
                    driftPercentage = this.lastDriftPercentage + Math.abs(this.lastNormalDriftPercentage - driftPercentage);
                    driftPercentage = Math.min(1, driftPercentage);
                }
                angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }

            double targetOmegaVelocity = this.omegaController.calculate(this.robot.getPosition().getRotation().getDegrees(), this.currentWaypoint.getHeading());
            omegaVelocity += MathUtil.clamp(targetOmegaVelocity - omegaVelocity, -this.constants.maxOmegaAccel * period, this.constants.maxOmegaAccel * period);
            omegaVelocity = MathUtil.clamp(omegaVelocity, -this.constants.maxOmegaVel, this.constants.maxOmegaVel);

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
        double distance = this.getDistanceToCurrentWaypoint();
        for (int i = this.getCurrentWaypointIndex(); i < this.waypoints.size() - 1; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getRawDistanceToFinalWaypoint() {
        return this.robot.getPosition().getTranslation().getDistance(this.getFinalWaypoint());
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

    public record Constants(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double maxDriftDistance, double maxSlowDistance, double finalSlowDistance) {}
}
