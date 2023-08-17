package obstacleavoiding.path;

import obstacleavoiding.math.BumbleUtil;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.constants.MovementConstants;
import obstacleavoiding.path.constants.PurePursuitConstants;
import obstacleavoiding.path.pid.PIDController;
import obstacleavoiding.path.robot.Robot;
import obstacleavoiding.path.robot.RobotState;
import obstacleavoiding.path.waypoints.NavigationWaypoint;
import obstacleavoiding.path.waypoints.Waypoint;
import obstacleavoiding.path.waypoints.WaypointAutoHeading;

import java.util.ArrayList;
import java.util.List;

/**
 * Class the controls the movement of the robot in a path of obstacles.
 * It follows a path that was created with some points.
 * He follows it with PID for a certain velocity that calculated by the distance
 * from the final waypoint, the curvature of the path
 */
public class PurePursuit {

    private static double MAX_VELOCITY;
    private static final double FF = 0;

    private final Robot robot;
    private final ObstacleAvoiding obstacleAvoiding;

    private final PIDController omegaController;

    private MovementConstants movementConstants;
    private PurePursuitConstants constants;
    private boolean isLinearRotation = true;

    private List<Waypoint> waypoints;
    private Waypoint currentWaypoint;
    private int currentWaypointIndex;

    private long lastUpdate;
    private double period;

    private Rotation2d drivingAngle;
    private double driveVelocity;
    private double omegaVelocity;

    private double originalDriftPercentage;
    private double driftPercentage;
    private double lastOriginalDriftPercentage;
    private double lastDriftPercentage;

    private double targetDriveVelocity;
    private double slowPercentage;

    private boolean isRunning = false;
    private boolean isFinished = false;

    public PurePursuit(ObstacleAvoiding obstacleAvoiding, Robot robot, PurePursuitConstants constants, MovementConstants movementConstants) {
        this.waypoints = new ArrayList<>();

        this.robot = robot;
        this.obstacleAvoiding = obstacleAvoiding;
        MAX_VELOCITY = this.robot.getConstants().maxVel();

        this.constants = constants;
        this.movementConstants = movementConstants;

        this.omegaController = new PIDController(3, 0, 0);
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset(boolean isLinearRotation, List<Waypoint> waypoints) {
        this.isFinished = false;

        this.waypoints = this.obstacleAvoiding.generateWaypoints(waypoints);
        this.currentWaypoint = this.waypoints.get(1);
        this.isLinearRotation = isLinearRotation;

        this.driveVelocity = this.getDrivetrainVelocity();

        this.lastUpdate = System.nanoTime();

        this.resetValues();
    }

    public void update() {
        this.update((System.nanoTime() - this.lastUpdate) / 1_000_000_000d);
    }

    public void update(double period) {
        this.period = period;
        this.currentWaypointIndex = this.getCurrentWaypointIndex();

        if (!this.isRunning()) return;

        this.drivingAngle = this.calculateDrivingAngle();
        this.driveVelocity = this.calculateVelocity();
        this.omegaVelocity = this.calculateOmega();

        double velocity = this.driveVelocity + FF;
        this.robot.drive(new Pose2d(
                new Translation2d(velocity, this.drivingAngle),
                Rotation2d.fromRadians(omegaVelocity)), this.period, false);

        if (this.currentWaypointIndex < this.waypoints.size() - 1 &&
                (driftPercentage >= 0.9 || this.isAbleToContinueNextWaypoint())) {
            this.currentWaypoint = this.waypoints.get(this.currentWaypointIndex + 1);
            this.resetValues();
        }

        if (this.isFinished()) {
            this.isFinished = true;
        }

        this.lastUpdate = System.nanoTime();
    }

    public boolean isFinished() {
        return this.driveVelocity <= this.constants.getVelocityTolerance() && this.getDistanceToFinalWaypoint() <= this.constants.getDistanceTolerance() &&
                Math.abs(this.robot.getPosition().getRotation().getDegrees() - this.getFinalWaypoint().getHeading().getDegrees()) <= this.constants.getRotationTolerance();
    }

    private boolean isAbleToContinueNextWaypoint() {
        return !(this.getCurrentWaypoint() instanceof NavigationWaypoint) &&
                !this.obstacleAvoiding.isObstacleDistributing(this.robot.getPosition().getTranslation(), this.getNextWaypointPosition()) &&
                BumbleUtil.bound360Degrees(this.drivingAngle.minus(this.robot.getPosition().getTranslation().minus(this.getNextWaypointPosition()).getAngle()).getDegrees()) <= 10 &&
                this.obstacleAvoiding.getObstacle(this.robot.getPosition().getTranslation()) == null;
    }

    private Rotation2d calculateDrivingAngle() {
        this.lastDriftPercentage = this.driftPercentage;
        this.lastOriginalDriftPercentage = this.originalDriftPercentage;

        Translation2d angle = normalize(this.getCurrentWaypointPosition().minus(this.robot.getPosition().getTranslation()));

        double maxDriftDistance = this.constants.getMaxDriftDistance();
        maxDriftDistance *= MathUtil.clamp(getPreviousWaypointPosition().getDistance(getCurrentWaypointPosition()) / (1.5 * maxDriftDistance), 0, 1);
        maxDriftDistance = Math.max(maxDriftDistance, 1);

        this.driftPercentage = 1 - (MathUtil.clamp(this.getDistanceToCurrentWaypoint() / maxDriftDistance, 0, 1));
        this.driftPercentage = Math.pow(this.driftPercentage, this.constants.getDriftPercentLinearity());
        this.originalDriftPercentage = this.driftPercentage;

        if (isNotLastWaypoint()) {
            Translation2d angleFromNext = normalize(this.getNextWaypointPosition().minus(this.robot.getPosition().getTranslation()));

            if (this.driftPercentage > 0 && this.lastDriftPercentage > this.driftPercentage) {
                this.driftPercentage = this.lastDriftPercentage + Math.abs(this.lastOriginalDriftPercentage - this.driftPercentage);
                this.driftPercentage = Math.min(driftPercentage, 1);
            }

            if (!(this.currentWaypoint instanceof NavigationWaypoint)) {
                angle = angle.times(1 - this.driftPercentage).plus(angleFromNext.times(this.driftPercentage));
            }
        }
        return angle.getAngle();
    }

    private double calculateVelocity() {
        List<Double> driftVelocity = new ArrayList<>();
        for (int i = this.currentWaypointIndex; i < this.waypoints.size() - 1; i++) {
            double turnAngle = this.getWaypointPosition(i + 1).minus(this.getWaypointPosition(i)).getAngle()
                    .minus(this.getWaypointPosition(i).minus(this.getWaypointPosition(i - 1)).getAngle()).getDegrees();
            turnAngle = Math.abs(BumbleUtil.bound180Degrees(turnAngle));

            double maxSlowDistance = (Math.pow(this.calculateDriftVelocity(turnAngle, 1), 2) - Math.pow(MAX_VELOCITY, 2)) / (2 * -this.constants.getDeceleration());

            double slowPercentage = 1 - (MathUtil.clamp(this.getDistanceToWaypoint(i) / maxSlowDistance, 0, 1));
            slowPercentage = Math.pow(slowPercentage, this.constants.getSlowPercentLinearity());

            this.slowPercentage = slowPercentage;

            driftVelocity.add(this.calculateDriftVelocity(turnAngle, slowPercentage));
        }

        double finalSlowDistance = Math.pow(MAX_VELOCITY, 2) / (2 * this.constants.getDeceleration());
        double stopVelocity = MathUtil.clamp(this.getDistanceToFinalWaypoint() / finalSlowDistance, 0, 1);
        stopVelocity = Math.pow(stopVelocity, 1 / this.constants.getFinalSlowPercentLinearity());
        stopVelocity *= MAX_VELOCITY;

        double targetDriveVelocity = Math.min(driftVelocity.stream().sorted().findFirst().orElse(this.movementConstants.getMaxVel()),
                stopVelocity);
        this.targetDriveVelocity = targetDriveVelocity;

        double velocity = this.driveVelocity;
        velocity += MathUtil.clamp(targetDriveVelocity - this.getDrivetrainVelocity(), -this.movementConstants.getMaxAccel() * period, this.movementConstants.getMaxAccel() * period);
        velocity = MathUtil.clamp(velocity, 0, targetDriveVelocity);

        return velocity;
    }

    private double calculateDriftVelocity(double angle, double slowPercentage) {
        double driftVelocity = angle * slowPercentage;
        driftVelocity = Math.pow((180 - driftVelocity) / 180, this.constants.getTurnVelocityLinearity());
        driftVelocity *= MAX_VELOCITY - this.constants.getMinimumDriftVelocity();
        driftVelocity += this.constants.getMinimumDriftVelocity();
        return driftVelocity;
    }

    private double calculateOmega() {
        int lastHeadingWaypoint = this.getLastHeadingWaypointIndex();
        int nextHeadingWaypoint = this.getNextHeadingWaypointIndex();

        double anglePercent = 1;
        if (this.isLinearRotation) {
             double absoluteDistance = this.getDistance(lastHeadingWaypoint, nextHeadingWaypoint);
             anglePercent = 1 - (this.getDistanceToWaypoint(nextHeadingWaypoint) / absoluteDistance);
             if (anglePercent >= 0.01) {
                 anglePercent = Math.pow(anglePercent, this.constants.getRotationPercentLinearity());
             }
        }

        double targetAngle = this.waypoints.get(lastHeadingWaypoint).getHeading().getRadians() +
                (anglePercent * BumbleUtil.boundPIRadians(this.waypoints.get(nextHeadingWaypoint).getHeading().minus(this.waypoints.get(lastHeadingWaypoint).getHeading()).getRadians()));
        targetAngle = BumbleUtil.boundPIRadians(targetAngle);

        double targetOmegaVelocity = this.omegaController.calculate(BumbleUtil.boundPIRadians(this.robot.getPosition().getRotation().getRadians()), targetAngle);
        double velocity = this.omegaVelocity;
        velocity += MathUtil.clamp(targetOmegaVelocity - velocity, -this.movementConstants.getMaxOmegaAccel() * period, this.movementConstants.getMaxOmegaAccel() * period);
        velocity = MathUtil.clamp(velocity, -this.movementConstants.getMaxOmegaVel(), this.movementConstants.getMaxOmegaVel());

        return velocity;
    }

    public void stop() {
        this.robot.drive(new Pose2d(), this.period, false);
    }

    private void resetValues() {
        this.driftPercentage = 0;
        this.lastDriftPercentage = 0;
        this.originalDriftPercentage = 0;
        this.lastOriginalDriftPercentage = 0;
    }

    private Translation2d normalize(Translation2d translation2d) {
        return translation2d.div(translation2d.getNorm() == 0 ? 1 : translation2d.getNorm());
    }

    public boolean isNotLastWaypoint() {
        return this.currentWaypointIndex < this.waypoints.size() - 1;
    }

    public double getDistance(int waypoint1, int waypoint2) {
        double distance = 0;
        for (int i = waypoint1; i < waypoint2; i++) {
            distance += this.waypoints.get(i).getDistance(this.waypoints.get(i + 1));
        }
        return distance;
    }

    public double getDistanceToWaypoint(int waypoint) {
        int currentWaypoint = this.getCurrentWaypointIndex();
        return this.getDistance(currentWaypoint, waypoint) +
                this.robot.getPosition().getTranslation().getDistance(this.getWaypointPosition(Math.min(currentWaypoint, this.waypoints.size() - 1)));
    }

    public double getDistanceToCurrentWaypoint() {
        return this.robot.getPosition().getTranslation().getDistance(this.getCurrentWaypointPosition());
    }

    public double getDistanceToFinalWaypoint() {
        return this.getDistanceToWaypoint(this.waypoints.size() - 1);
    }

    public Waypoint getCurrentWaypoint() {
        return this.waypoints.get(this.currentWaypointIndex);
    }

    public Waypoint getFinalWaypoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public Translation2d getCurrentWaypointPosition() {
        return this.currentWaypoint.getReferencedPosition();
    }

    public Translation2d getPreviousWaypointPosition() {
        return this.getWaypointPosition(this.currentWaypointIndex - 1);
    }

    public Translation2d getNextWaypointPosition() {
        return this.getWaypointPosition(this.currentWaypointIndex + 1);
    }

    public Translation2d getWaypointPosition(int index) {
        return this.waypoints.get(index).getReferencedPosition();
    }

    private double getDrivetrainVelocity() {
        return this.robot.getVelocity().getTranslation().getNorm();
    }

    public Rotation2d getDrivingAngle() {
        return this.drivingAngle;
    }

    public int getCurrentWaypointIndex() {
        for (int i = 1; i < this.waypoints.size(); i++) {
            if (this.waypoints.get(i).equals(this.currentWaypoint))
                return i;
        }
        this.currentWaypoint = this.waypoints.get(1);
        return 1;
    }

    private int getNextHeadingWaypointIndex() {
        for (int i = this.getCurrentWaypointIndex(); i < this.waypoints.size(); i++) {
            if (!(this.waypoints.get(i) instanceof WaypointAutoHeading))
                return i;
        }
        return -1;
    }

    private int getLastHeadingWaypointIndex() {
        int current = this.getCurrentWaypointIndex();
        for (int i = current - 1; i >= 0; i--) {
            if (!(this.waypoints.get(i) instanceof WaypointAutoHeading))
                return i;
        }
        return -1;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public PurePursuitConstants getConstants() {
        return constants;
    }

    public void setConstants(PurePursuitConstants constants) {
        this.constants = constants;
    }

    public MovementConstants getMovementConstants() {
        return movementConstants;
    }

    public void setMovementConstants(MovementConstants movementConstants) {
        this.movementConstants = movementConstants;
    }

    public boolean isRunning() {
        return this.isRunning && !this.isFinished;
    }

    public void setRunning(boolean isRunning) {
        if (!this.isRunning && isRunning) {
            this.lastUpdate = System.nanoTime();
        }
        this.isRunning = isRunning;
    }

    public double getTargetDriveVelocity() {
        return targetDriveVelocity;
    }

    public double getDriftPercentage() {
        return driftPercentage;
    }

    public double getSlowPercentage() {
        return slowPercentage;
    }

    public List<RobotState> getEstimatedPath(double period) {
        List<RobotState> poses = new ArrayList<>();
        Robot robot = new Robot(this.waypoints.get(0).getPose2d(), this.robot.getConstants());
        PurePursuit purePursuit = new PurePursuit(this.obstacleAvoiding, robot, this.constants, this.movementConstants);
        purePursuit.reset(this.isLinearRotation, new ArrayList<>(this.waypoints));
        purePursuit.setRunning(true);
        while (!purePursuit.isFinished()) {
            purePursuit.update(period);
            poses.add(new RobotState(robot.getPosition(), robot.getVelocity().getTranslation().getNorm(),
                    MathUtil.limitDot(purePursuit.driftPercentage, 3), MathUtil.limitDot(purePursuit.slowPercentage, 3),
                    MathUtil.limitDot(purePursuit.getDistanceToCurrentWaypoint(), 3)));
        }
        return poses;
    }
}
