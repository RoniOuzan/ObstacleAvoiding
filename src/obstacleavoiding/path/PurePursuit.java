package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDPreset;
import obstacleavoiding.path.pid.ProfiledPIDController;
import obstacleavoiding.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PurePursuit {
    private final List<Waypoint> waypoints;

    private final ProfiledPIDController driveController;
    private final ProfiledPIDController omegaController;

    private final Robot robot;

    private int currentWaypoint = 1;
    private boolean isFinished = false;
    private boolean isRunning = true;

    public PurePursuit(Robot robot, Constants constants, List<Waypoint> waypoints) {
        this.robot = robot;
        this.waypoints = waypoints;

        this.driveController = new ProfiledPIDController(constants.drivePreset);
        this.omegaController = new ProfiledPIDController(constants.omegaPreset);
    }

    public PurePursuit(Robot robot, Constants constants, Waypoint... waypoints) {
        this(robot, constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public void update() {
        if (this.isRunning && !this.isFinished) {
            Pose2d velocity = new Pose2d(
                    new Translation2d(
                            this.driveController.calculate(this.getDistanceToFinalWaypoint()),
                            this.robot.getPosition().getTranslation().minus(this.getCurrentWaypoint()).getAngle()
                    ),
                    Rotation2d.fromDegrees(
                            this.omegaController.calculate(this.robot.getPosition().getRotation().getDegrees(),
                                    this.getCurrentWaypoint().getHeading()))
            );

            this.robot.drive(velocity);

            if (this.getCurrentWaypoint().isPassedWaypoint(this.robot)) {
                this.currentWaypoint++;
                if (this.currentWaypoint >= this.waypoints.size()) {
                    this.isFinished = true;
                }
            }
        }
    }

    public void reset() {
        this.currentWaypoint = 1;
        this.robot.setPosition(new Pose2d(this.getStartWaypoint(), Rotation2d.fromDegrees(this.getStartWaypoint().getHeading())));
        this.resetControllers();
        this.isFinished = false;
    }

    private void resetControllers() {
        this.driveController.setGoal(0);
        this.driveController.reset(this.getDistanceToFinalWaypoint());
        this.omegaController.reset(this.robot.getPosition().getRotation().getDegrees());
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

    public ProfiledPIDController getDriveController() {
        return driveController;
    }

    public ProfiledPIDController getOmegaController() {
        return omegaController;
    }

    public void setRunning(boolean running) {
        this.isRunning = running;
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

    public record Constants(PIDPreset drivePreset, PIDPreset omegaPreset) {}
}
