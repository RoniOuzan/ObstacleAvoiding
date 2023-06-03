package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Transform2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.pid.PIDPreset;
import obstacleavoiding.path.pid.ProfiledPIDController;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PurePursuit {

    public static double FF_MIN_DISTANCE_REDUCER = 0.2;
    public static double FF_MAX_DISTANCE_REDUCER = 1.6;

    public static double FF_MIN_OMEGA_REDUCER = 5;
    public static double FF_MAX_OMEGA_REDUCER = 30;

    private List<Waypoint> waypoints;

    private final PIDController driveController;
    private final ProfiledPIDController omegaController;

    private final Constants constants;

    private final Robot robot;

    private double velocity;

    private double lastDriftPercentage;

    private int currentWaypoint = 1;
    private boolean isFinished = false;
    private boolean isRunning = true;

    private long lastUpdate;

    public PurePursuit(Robot robot, Constants constants, List<Waypoint> waypoints) {
        this.robot = robot;
        this.constants = constants;
        this.waypoints = waypoints;

        this.driveController = new PIDController(0.3, 0, 0);
        this.omegaController = new ProfiledPIDController(constants.omegaPreset);
        this.omegaController.enableContinuousInput(-180, 180);
    }

    public PurePursuit(Robot robot, Constants constants, Waypoint... waypoints) {
        this(robot, constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public void reset() {
        this.currentWaypoint = 1;
        this.robot.setPosition(new Pose2d(this.getStartWaypoint(), Rotation2d.fromDegrees(this.getStartWaypoint().getHeading())));
        this.robot.drive(new Pose2d());
        this.velocity = 0;
        this.lastUpdate = System.nanoTime();

        this.resetControllers();
        this.isFinished = false;
    }

    private void resetControllers() {
        this.omegaController.reset(this.robot.getPosition().getRotation().getDegrees());
    }

    public void update() {
        if (this.isRunning && !this.isFinished) {
            double period = (System.nanoTime() - this.lastUpdate) / 1_000_000_000d;

            Translation2d angle = this.getCurrentWaypoint().minus(this.robot.getPosition().getTranslation()).normalized();
            double driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxDriftDistance) / this.constants.maxDriftDistance);
            double slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint(), 0, this.constants.maxSlowDistance) / this.constants.maxSlowDistance);

            driftPercentage = Math.pow(driftPercentage, 1.5);

            if (this.currentWaypoint < this.waypoints.size() - 1) {
                Translation2d angleFromNext = this.getWaypoint(this.currentWaypoint + 1).minus(this.robot.getPosition().getTranslation()).normalized();

                angle = angle.times(1 - driftPercentage).plus(angleFromNext.times(driftPercentage));
            }

            double targetVelocity = this.getDistanceToFinalWaypoint() < 1.2 ? 0 : this.constants.maxVel - (slowPercentage * 2);
            velocity += MathUtil.clamp(this.driveController.calculate(this.velocity, targetVelocity), -this.constants.maxAccel * period, this.constants.maxAccel * period);

            velocity = MathUtil.clamp(velocity, 0, this.constants.maxVel - (driftPercentage * 2));

            this.robot.drive(new Pose2d(
                    new Translation2d(velocity, angle.getAngle()),
                    Rotation2d.fromDegrees(0)));

            if (driftPercentage >= 0.5 && this.lastDriftPercentage > driftPercentage) {
                if (this.currentWaypoint < this.waypoints.size() - 1) {
                    this.currentWaypoint++;
                } else if (this.getDistanceToCurrentWaypoint() <= 0.03 && Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading()) <= 1) {
                    this.isFinished = true;
                }
            }

            this.lastUpdate = System.nanoTime();
            this.lastDriftPercentage = driftPercentage;
        }
    }

    public double getDistanceToCurrentWaypoint() {
        return this.robot.getPosition().getTranslation().getDistance(this.getCurrentWaypoint());
    }

    public double getDistanceToFinalWaypoint() {
        double distance = this.getDistanceToCurrentWaypoint();
        for (int i = this.currentWaypoint; i < this.waypoints.size() - 1; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getAbsoluteDistance() {
        double distance = 0;
        for (int i = 0; i < this.waypoints.size() - 1; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getDriftPercentage() {
        return lastDriftPercentage;
    }

    public ProfiledPIDController getDriveController() {
        return new ProfiledPIDController(new PIDPreset(0, 0, 0, 0, 0));
    }

    public ProfiledPIDController getOmegaController() {
        return omegaController;
    }

    public void setRunning(boolean running) {
        this.isRunning = running;
        if (isRunning) {
            this.lastUpdate = System.nanoTime();
        }
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public Waypoint getCurrentWaypoint() {
        return this.getWaypoint(this.currentWaypoint);
    }

    public Waypoint getPreviousWaypoint() {
        return this.getWaypoint(this.currentWaypoint - 1);
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

    public void setWaypoint(int index, Waypoint waypoint) {
        this.waypoints.set(index, waypoint);
    }

    public void setWaypoints(List<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public record Constants(double maxVel, double maxAccel, double maxDriftDistance, double maxSlowDistance, PIDPreset drivePreset, PIDPreset omegaPreset) {}
}
