package obstacleavoiding.path;

import com.github.strikerx3.jxinput.XInputComponents;
import com.github.strikerx3.jxinput.XInputDevice;
import obstacleavoiding.gui.Frame;
import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroLeftBottom;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.*;
import obstacleavoiding.path.constants.MovementConstants;
import obstacleavoiding.path.constants.PurePursuitConstants;
import obstacleavoiding.path.fields.Fields;
import obstacleavoiding.path.obstacles.DraggableObstacle;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.robot.ModuleState;
import obstacleavoiding.path.robot.Robot;
import obstacleavoiding.path.robot.RobotState;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.settings.WaypointSettings;
import obstacleavoiding.path.settings.tables.*;
import obstacleavoiding.path.util.Alliance;
import obstacleavoiding.path.util.Bounds;
import obstacleavoiding.path.util.ValuesMode;
import obstacleavoiding.path.waypoints.NavigationWaypoint;
import obstacleavoiding.path.waypoints.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.List;
import java.util.*;
import java.util.function.Supplier;

public class GUI extends Frame implements ZeroLeftBottom, DrawCentered {
    public static final Color COLOR = new Color(245, 212, 9); // #F5D409

    public static final int SETTINGS_WIDTH = 200;

    public static final double DEFAULT_MAX_VALUE = 16.54; // 8.27
    public static final double SCREEN_MULTIPLIER = 1;
    public static final Dimension2d FIELD_DIMENSION = new Dimension2d(1713, 837).times(SCREEN_MULTIPLIER);
    public static final Dimension2d FRAME_DIMENSION = FIELD_DIMENSION.plus(new Dimension2d(SETTINGS_WIDTH, 0));
    public static final double DEFAULT_MAX_Y = DEFAULT_MAX_VALUE * ((double) FIELD_DIMENSION.getY() / FIELD_DIMENSION.getX());

    private static final int GRAPH_HISTORY = 0;

    private static final double FPS = 50;
    public static final double ROBOT_SIZE = 0.91;
    public static final double HALF_ROBOT = ROBOT_SIZE / 2;
    private static final double TRACK_WIDTH = 0.6;
    private static final double WHEEL_BASE = 0.6;
    private static final double WHEEL_WIDTH = 0.05;
    private static final double WHEEL_DIAMETER = 0.1;

    private static final double MINI_ROBOT_SIZE = ROBOT_SIZE;

    private static final Fields DEFAULT_FIELD = Fields.CHARGED_UP;
    private static final Alliance DEFAULT_ALLIANCE = Alliance.NONE;

    private static final String IMAGES_PATH = "src/main/java/obstacleavoiding/path/images/";

    private final Settings settings;
    private final WaypointSettings waypointSettings;

    private final ObstacleAvoiding obstacleAvoiding;
    private final List<Waypoint> defaultWaypoints;
    private final PurePursuit purePursuit;
    private final Robot robot;

    private List<RobotState> estimatedPath;

    private Image robotImage = new ImageIcon(IMAGES_PATH + DEFAULT_ALLIANCE.getText() + "Robot.png").getImage();
    private Image invisibleRobotImage = new ImageIcon(IMAGES_PATH + DEFAULT_ALLIANCE.getText() + "InvisibleRobot.png").getImage();

    private boolean isControllerDrive = true;

    private final Map<Pose2d, Integer> positions = new HashMap<>();

    private final List<Double> currentWaypoint = new ArrayList<>();
    private final List<Double> driveVelocities = new ArrayList<>();
    private final List<Double> targetDriveVelocities = new ArrayList<>();
    private final List<Double> driftPercentageVelocities = new ArrayList<>();
    private final List<Double> slowPercentages = new ArrayList<>();
    private final List<Double> omegaVelocities = new ArrayList<>();
    private final List<Double> drivingAngles = new ArrayList<>();
    private final List<Double> robotAngle = new ArrayList<>();
    private final List<Double> distance = new ArrayList<>();
    private final List<Double> curvatures = new ArrayList<>();

    private int graphHistory = GRAPH_HISTORY;

    private double maxValue = DEFAULT_MAX_VALUE;

    private Waypoint draggedWaypoint = null;
    private Obstacle draggedObstacle = null;

    private Waypoint selectedWaypoint = null;

    public GUI() {
        super("Path Follower", FRAME_DIMENSION, FPS);
        this.addDevice(0);

        this.robot = new obstacleavoiding.path.robot.Robot(new Pose2d(new Translation2d(12.43, 4.89), Rotation2d.fromDegrees(0)),
                new Robot.Constants(4.5, 100, 360, 720, 1 / FPS, getModulesLocation()));

        this.obstacleAvoiding = new ObstacleAvoiding(HALF_ROBOT + 0.01, new Bounds(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y), DEFAULT_FIELD.getField().getObstacles());

        this.defaultWaypoints = new ArrayList<>();
        this.defaultWaypoints.add(new Waypoint(this.robot.getPosition(), Waypoint.RobotReferencePoint.CENTER));
//        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 2, DEFAULT_MAX_Y / 2, Rotation2d.fromDegrees(0), Waypoint.RobotReferencePoint.CENTER));
        this.defaultWaypoints.add(new Waypoint(14.54, 2.85, Rotation2d.fromDegrees(0), Waypoint.RobotReferencePoint.CENTER));

        this.purePursuit = new PurePursuit(
                this.obstacleAvoiding,
                this.robot,
                new PurePursuitConstants(
                        1, 1.5,
                        2, 1,
                        1, 0.5,
                        6, 0,
                        0.03, 0.1, 0.7),
                new MovementConstants(4.5, 8, 360, 360)
                );

        Map<Supplier<List<Double>>, Color> graphValues = new HashMap<>();
        graphValues.put(() -> driftPercentageVelocities, new Color(0, 0, 255));
        graphValues.put(() -> currentWaypoint, new Color(0, 255, 0));
        graphValues.put(() -> driveVelocities.stream().map(s -> s / this.robot.getConstants().maxVel()).toList(), new Color(255, 0, 0));
        graphValues.put(() -> targetDriveVelocities.stream().map(s -> s / this.robot.getConstants().maxVel()).toList(), new Color(100, 0, 0));
        graphValues.put(() -> omegaVelocities.stream().map(s -> (s + 180) / this.robot.getConstants().maxOmegaVel()).toList(), new Color(255, 255, 0));
        graphValues.put(() -> drivingAngles.stream().map(s -> MathUtil.inputModulus(s / 360 + 0.5, 0, 1)).toList(), new Color(0, 255, 255));
        graphValues.put(() -> robotAngle.stream().map(s -> MathUtil.inputModulus(s / 360 + 0.5, 0, 1)).toList(), new Color(135, 206, 235));
        graphValues.put(() -> slowPercentages, new Color(120, 120, 150));

        this.addGraph("Values", graphValues, 0, 1);

        List<TableType<?>> values = new ArrayList<>();
        values.add(new SelectOptionTable<>("Field", ValuesMode.SHOWCASE, DEFAULT_FIELD, Fields.values()).onChange((c, l) -> {
            this.obstacleAvoiding.setObstacles(c.getField().getObstacles());
            this.reset();
        }));
        values.add(new SelectOptionTable<>("Alliance", ValuesMode.SHOWCASE, DEFAULT_ALLIANCE, Alliance.values()).onChange((c, l) -> {
            robotImage = new ImageIcon(IMAGES_PATH + c.getText() + "Robot.png").getImage();
            invisibleRobotImage = new ImageIcon(IMAGES_PATH + c.getText() + "InvisibleRobot.png").getImage();
        }));
        values.add(new DoubleSliderTable("FPS", ValuesMode.SHOWCASE, FPS, 1, 50).setValueParser(this::setFPS));
        values.add(new DoubleSliderTable("MaxVel", ValuesMode.ALL, this.purePursuit.getMovementConstants().getMaxVel(), 0, 5).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("MaxAccel", ValuesMode.ALL, this.purePursuit.getMovementConstants().getMaxAccel(), 0, 20).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("MaxOmegaVel", ValuesMode.ALL, Math.toDegrees(this.purePursuit.getMovementConstants().getMaxOmegaVel()), 0, 720).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("MaxOmegaAccel", ValuesMode.ALL, Math.toDegrees(this.purePursuit.getMovementConstants().getMaxOmegaAccel()), 0, 1080).onChange((l, c) -> this.estimatePath()));

        values.add(new DoubleSliderTable("MaxDriftDistance", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getMaxDriftDistance(), 0, 5).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("DriftPercentLinearity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getDriftPercentLinearity(), 0, 5).onChange((l, c) -> this.estimatePath()));

        values.add(new DoubleSliderTable("TurnVelocityLinearity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getTurnVelocityLinearity(), 0, 5).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("SlowPercentLinearity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getSlowPercentLinearity(), 0, 5).onChange((l, c) -> this.estimatePath()));

        values.add(new DoubleSliderTable("FinalSlowLinearity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getFinalSlowPercentLinearity(), 0, 5).onChange((l, c) -> this.estimatePath()));

        values.add(new DoubleSliderTable("RotationPercentLinearity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getRotationPercentLinearity(), 0, 5).onChange((l, c) -> this.estimatePath()));

        values.add(new DoubleSliderTable("Deceleration", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getDeceleration(), 0, 20).onChange((l, c) -> this.estimatePath()));
        values.add(new DoubleSliderTable("MinimumDriftVelocity", ValuesMode.CALIBRATION, this.purePursuit.getConstants().getMinimumDriftVelocity(), 0, 4.5).onChange((l, c) -> this.estimatePath()));

        values.add(new IntegerSliderTable("GraphHistory", ValuesMode.SHOWCASE, GRAPH_HISTORY, 0, 200).setValueParser(d -> graphHistory = d));
        values.add(new BooleanTable("Reset", ValuesMode.ALL, false).onTrue(this::reset).setAlways(false));
        values.add(new BooleanTable("Running", ValuesMode.ALL, true).setValueParser(this.purePursuit::setRunning));
        values.add(new BooleanTable("Extended Obstacles", ValuesMode.SHOWCASE, false));
        values.add(new BooleanTable("Linear Rotation", ValuesMode.ALL, true).onTrue(this::resetPath));
        values.add(new BooleanTable("Auto Generate", ValuesMode.SHOWCASE, false).onTrue(this::resetPath));
        values.add(new BooleanTable("Trail", ValuesMode.SHOWCASE, true));
        values.add(new BooleanTable("Path", ValuesMode.SHOWCASE, true));
        values.add(new BooleanTable("EstimatedPath", ValuesMode.ALL, false));
        values.add(new BooleanTable("Create Waypoint", ValuesMode.ALL, false).setAlways(false).onTrue(() -> {
                    this.defaultWaypoints.add(this.defaultWaypoints.size() - 1, new NavigationWaypoint(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2, new Rotation2d(), Waypoint.RobotReferencePoint.CENTER, 1));
                    this.resetPath();
                }));
        this.settings = new Settings(values);
        this.add(this.settings);

        this.waypointSettings = new WaypointSettings(this.robot);
        this.add(this.waypointSettings);

        this.purePursuit.reset(this.settings.getValue("Linear Rotation", true), this.defaultWaypoints);
        this.start();
    }

    public void drawBackground() {
        this.drawImage(this.settings.getValue("Field", DEFAULT_FIELD).getField().getImage(), convertX(0, FIELD_DIMENSION), (int) (convertY(0, FIELD_DIMENSION) - convertUnits(DEFAULT_MAX_Y)),
                (int) convertUnits(DEFAULT_MAX_VALUE), (int) convertUnits(DEFAULT_MAX_Y));
    }

    public void displayPath() {
        if (!this.settings.getValue("Path", true)) return;

        for (Waypoint waypoint : this.purePursuit.getWaypoints()) {
            this.drawImage(invisibleRobotImage,
                    waypoint.getReferencedPosition(),
                    ROBOT_SIZE, ROBOT_SIZE,
                    -waypoint.getHeading().getDegrees());

            if (this.selectedWaypoint == waypoint) {
                this.drawSelectedWaypoint(waypoint.getTranslation2d());
            } else {
                this.drawWaypoint(waypoint.getTranslation2d());
            }
        }
        this.drawConnectedPoints(Color.BLACK, this.purePursuit.getWaypoints().parallelStream().map(Waypoint::getReferencedPosition).toList());

        if (this.settings.getValue("EstimatedPath", false)) {
            for (RobotState state : this.estimatedPath) {
                this.fillPoint(state.pose().getTranslation(), convertPixelsToUnits(3),
                        new Color((float) (state.velocity() / this.robot.getConstants().maxVel()),
                                0,
                                0f));
            }
        }
    }

    public void displayTrail() {
        if (!this.settings.getValue("Trail", true)) return;

        for (Map.Entry<Pose2d, Integer> entry : this.positions.entrySet()) {
            this.drawImage(invisibleRobotImage,
                    entry.getKey().getTranslation(),
                    MINI_ROBOT_SIZE, MINI_ROBOT_SIZE,
                    -entry.getKey().getRotation().getDegrees());
        }
    }

    public void displayRobot() {
        this.drawImage(robotImage,
                this.robot.getPosition().getTranslation(),
                ROBOT_SIZE, ROBOT_SIZE,
                -this.robot.getPosition().getRotation().getDegrees());

        for (ModuleState module : this.robot.getModules()) {
            this.fillAngledRect(
                    this.robot.getPosition().getTranslation().plus(new Translation2d(
                    module.getLocation().getNorm(), module.getLocation().getAngle().plus(this.robot.getPosition().getRotation()))),
                    WHEEL_DIAMETER, WHEEL_WIDTH, module.getAngle().plus(this.robot.getPosition().getRotation()).getDegrees(), Color.DARK_GRAY.darker());
        }
    }

    public void displayObstacles() {
        for (Obstacle obstacle : this.obstacleAvoiding.getObstacles()) {
            this.fillPolygon(obstacle.getAlliance().getColor(50), obstacle.getCorners()
                    .parallelStream().map(c -> new Translation2d(Math.min(c.getX(), DEFAULT_MAX_VALUE), Math.min(c.getY(), DEFAULT_MAX_Y))).toList());
            if (obstacle instanceof DraggableObstacle) {
                this.fillPoint(obstacle.getCenter(), convertPixelsToUnits(7), Color.RED);
            }
        }
        if (this.settings.getValue("Extended Obstacles", false)) {
            this.obstacleAvoiding.getObstacles().stream().map(o -> o.getExtendedObstacle(this.obstacleAvoiding.getDistanceThreshold()))
                    .forEach(o -> this.fillPolygon(new Color(0, 0, 0, 100), o.getCorners()
                            .parallelStream().map(c -> new Translation2d(Math.min(c.getX(), DEFAULT_MAX_VALUE), Math.min(c.getY(), DEFAULT_MAX_Y))).toList()));
        }
    }

    public void writeValues() {
        String[] texts = {
                "Pose: (" + MathUtil.limitDot(this.robot.getPosition().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getPosition().getTranslation().getY(), 3) + ")",
                "Error: " + this.purePursuit.getDistanceToCurrentWaypoint(),
                "Absolute Error: " + this.purePursuit.getDistanceToFinalWaypoint(),
                "Heading: " + MathUtil.limitDot(this.robot.getPosition().getRotation().getDegrees(), 3) + " Deg",
                "Vector: (" + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getY(), 3) + ")",
                "Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getNorm(), 3) + "m/s",
                "Acceleration: " + MathUtil.limitDot(this.robot.getAcceleration(), 3) + "m/s",
                "Omega Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getRotation().getDegrees(), 3) + " deg/s",
        };

        double size = convertPixelsToUnits(20);
        double space = convertPixelsToUnits(10);
        for (int i = 0; i < texts.length; i++) {
            this.write(convertPixelsToUnits(5), DEFAULT_MAX_Y - ((size + space) * (i + 1)), texts[i], size, Color.BLACK);
        }
    }

    public void updateValues() {
        if (this.purePursuit.isRunning()) {
            this.positions.put(this.robot.getPosition(), this.purePursuit.getCurrentWaypointIndex());
        }

        if (this.graphHistory > 0 || this.purePursuit.isRunning()) {
            this.currentWaypoint.add(this.purePursuit.getCurrentWaypointIndex() * 1d / this.purePursuit.getWaypoints().size());
            this.driveVelocities.add(this.robot.getVelocity().getTranslation().getNorm());
            this.targetDriveVelocities.add(this.purePursuit.getTargetDriveVelocity());
            this.driftPercentageVelocities.add(this.purePursuit.getDriftPercentage());
            this.slowPercentages.add(this.purePursuit.getSlowPercentage());
            this.omegaVelocities.add(this.robot.getVelocity().getRotation().getDegrees());
            this.drivingAngles.add(this.robot.getVelocity().getTranslation().getAngle().getDegrees());
            this.robotAngle.add(this.robot.getPosition().getRotation().bound(0, 360).getDegrees());
        }

        if (this.graphHistory > 0) {
            limitList(this.currentWaypoint);
            limitList(this.driveVelocities);
            limitList(this.targetDriveVelocities);
            limitList(this.driftPercentageVelocities);
            limitList(this.slowPercentages);
            limitList(this.omegaVelocities);
            limitList(this.drivingAngles);
            limitList(this.robotAngle);
            limitList(this.distance);
            limitList(this.curvatures);
        }
    }

    private void limitList(List<?> list) {
        while (list.size() > this.graphHistory)
            list.remove(0);
    }

    public void resetPath() {
        this.defaultWaypoints.set(0, new Waypoint(this.robot.getPosition().getTranslation(), this.robot.getPosition().getRotation(), Waypoint.RobotReferencePoint.CENTER));
        this.purePursuit.reset(this.settings.getValue("Linear Rotation", true), this.defaultWaypoints);

        this.estimatePath();
    }

    public void estimatePath() {
        this.estimatedPath = this.purePursuit.getEstimatedPath(1 / this.settings.getValue("FPS", 1d));
    }

    public void reset() {
        this.resetPath();

        this.positions.clear();

        if (this.graphHistory <= 0) {
            this.currentWaypoint.clear();
            this.driveVelocities.clear();
            this.targetDriveVelocities.clear();
            this.driftPercentageVelocities.clear();
            this.slowPercentages.clear();
            this.omegaVelocities.clear();
            this.drivingAngles.clear();
            this.robotAngle.clear();
            this.distance.clear();
            this.curvatures.clear();
        }

        this.purePursuit.setRunning(true);
    }

    @Override
    public void update() {
        this.purePursuit.setConstants(new PurePursuitConstants(
                this.settings.getValue("MaxDriftDistance", 0d), this.settings.getValue("DriftPercentLinearity", 0d),
                this.settings.getValue("TurnVelocityLinearity", 0d), this.settings.getValue("SlowPercentLinearity", 0d),
                this.settings.getValue("FinalSlowLinearity", 0d), this.settings.getValue("RotationPercentLinearity", 0d),
                this.settings.getValue("Deceleration", 0d), this.settings.getValue("MinimumDriftVelocity", 0d),
                this.purePursuit.getConstants().getDistanceTolerance(), this.purePursuit.getConstants().getVelocityTolerance(), this.purePursuit.getConstants().getRotationTolerance()
        ));
        this.purePursuit.setMovementConstants(new MovementConstants(
                this.settings.getValue("MaxVel", 0d), this.settings.getValue("MaxAccel", 0d),
                this.settings.getValue("MaxOmegaVel", 0d), this.settings.getValue("MaxOmegaAccel", 0d)
        ));
        this.drawBackground();
        this.displayPath();
        this.settings.update();
        this.waypointSettings.update(this.selectedWaypoint);
        this.purePursuit.update();
        this.updateValues();
        this.displayTrail();
        this.displayObstacles();
        this.writeValues();
        this.displayRobot();
    }

    @Override
    public double getPixelsInUnits() {
        return FIELD_DIMENSION.getX() / this.maxValue;
    }

    @Override
    public void deviceListen(XInputDevice device, XInputComponents components, XInputComponents lastComponents) {
        if (components.getButtons().a && !lastComponents.getButtons().a) {
            this.settings.setValue("Reset", true);
        } else if (components.getButtons().b && !lastComponents.getButtons().b) {
            this.settings.setValue("Running", !(boolean) this.settings.getValue("Running", false));
        }

        if (this.purePursuit.isRunning())
            return;

        this.isControllerDrive = true;

        Translation2d leftAxis = new Translation2d(
                MathUtil.limitDot(Math.max(components.getAxes().lx, 0.05), 3),
                MathUtil.limitDot(Math.max(components.getAxes().ly, 0.05), 3)
        );
        double rightAxis = Math.toDegrees(-Math.pow(MathUtil.limitDot(Math.max(components.getAxes().rx, 0.05), 3), 1) * this.robot.getConstants().maxVel());

        Rotation2d angle = leftAxis.getNorm() < 0.01 ? this.robot.getVelocity().getTranslation().getAngle() : leftAxis.getAngle();
        double magnitude = Math.pow(Math.min(leftAxis.getNorm(), 1), 2) * this.robot.getConstants().maxVel();

        Translation2d velocity = new Translation2d(magnitude, angle);

        if (this.obstacleAvoiding.getObstacle(this.robot.getPosition().getTranslation().plus(velocity.times(this.getPeriod()))) != null)
            return;

        this.robot.drive(new Pose2d(velocity, Rotation2d.fromDegrees(rightAxis)), getPeriod(), true);

        if (this.settings.getValue("Auto Generate", false) && this.robot.isMoving()) {
            this.resetPath();
        }
    }

    @Override
    public void mouseDragged(MouseEvent e, Translation2d mouseLocation) {
        if (mouseLocation.getX() > this.maxValue)
            return;

        boolean drag = false;
        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0 && !drag; i--) {
            Waypoint waypoint = this.purePursuit.getWaypoints().get(i);
            if (waypoint == this.draggedWaypoint) {
                waypoint.set(mouseLocation);

                if (this.settings.getValue("Auto Generate", false)) {
                    this.resetPath();
                }

                drag = true;
            }
        }

        for (int i = this.obstacleAvoiding.getObstacles().size() - 1; i >= 0 && !drag; i--) {
            Obstacle obstacle = this.obstacleAvoiding.getObstacles().get(i);
            if (obstacle == this.draggedObstacle) {
                obstacle.setCorners(DraggableObstacle.getCornersOfObstacle(mouseLocation, obstacle));

                if (this.settings.getValue("Auto Generate", false)) {
                    this.resetPath();
                }

                drag = true;
            }
        }
    }

    @Override
    public void keyListen(Set<Integer> keys) {
        double omega = this.robot.getConstants().maxVel();
        Pose2d velocity = new Pose2d();
        if (keys.contains(KeyEvent.VK_W))
            velocity = velocity.plus(new Transform2d(new Translation2d(0, 1), new Rotation2d()));
        if (keys.contains(KeyEvent.VK_S))
            velocity = velocity.plus(new Transform2d(new Translation2d(0, -1), new Rotation2d()));
        if (keys.contains(KeyEvent.VK_A))
            velocity = velocity.plus(new Transform2d(new Translation2d(-1, 0), new Rotation2d()));
        if (keys.contains(KeyEvent.VK_D))
            velocity = velocity.plus(new Transform2d(new Translation2d(1, 0), new Rotation2d()));
        if (keys.contains(KeyEvent.VK_E))
            velocity = new Pose2d(velocity.getTranslation(), velocity.getRotation().minus(Rotation2d.fromRadians(omega)));
        if (keys.contains(KeyEvent.VK_Q))
            velocity = new Pose2d(velocity.getTranslation(), velocity.getRotation().plus(Rotation2d.fromRadians(omega)));

        if (velocity.getTranslation().getNorm() >= 0.01 || Math.abs(velocity.getRotation().getDegrees()) > 0.5)
            this.isControllerDrive = false;

        if (this.purePursuit.isRunning() || this.isControllerDrive || keys.size() == 0)
            return;

        this.robot.drive(new Pose2d(
                velocity.getTranslation().normalized().times(this.robot.getConstants().maxVel()),
                velocity.getRotation()), getPeriod(), false);

        if (this.settings.getValue("Auto Generate", false) && this.robot.isMoving()) {
            this.resetPath();
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_R) {
            this.settings.setValue("Reset", true);
        } else if (e.getKeyCode() == KeyEvent.VK_T) {
            this.settings.setValue("Running", !this.purePursuit.isRunning());
        } else if (e.getKeyCode() == KeyEvent.VK_G) {
            this.settings.setValue("Extended Obstacles", !this.settings.getValue("Extended Obstacles", false));
        } else if (e.getKeyCode() == KeyEvent.VK_Y) {
            this.settings.setValue("Auto Generate", !this.settings.getValue("Auto Generate", false));
        } else if (e.getKeyCode() == KeyEvent.VK_C) {
            this.settings.setValue("EstimatedPath", !this.settings.getValue("EstimatedPath", false));
        }

        if (this.selectedWaypoint != null) {
            if (e.getKeyChar() == KeyEvent.VK_BACK_SPACE && this.defaultWaypoints.size() > 2) {
                this.purePursuit.getWaypoints().remove(this.selectedWaypoint);
                this.defaultWaypoints.remove(this.selectedWaypoint);
                this.selectedWaypoint = null;

                this.resetPath();
            }
        }
    }

    @Override
    public void mouseClicked(MouseEvent e, Translation2d mouseLocation) {
        if (mouseLocation.getX() > this.maxValue)
            return;

        Waypoint waypoint = this.getWaypointOnMouse(mouseLocation);

        if (waypoint != null) {
            if (this.selectedWaypoint == waypoint) {
                this.selectedWaypoint = null;
            } else {
                this.selectedWaypoint = waypoint;
            }
        } else if (this.selectedWaypoint != null) {
            this.selectedWaypoint = null;
        } else if (!this.settings.getValue("EstimatedPath", false)) {
            Waypoint lastWaypoint = this.defaultWaypoints.get(this.defaultWaypoints.size() - 1);
            lastWaypoint.set(mouseLocation);
            this.reset();
        }

        if (this.settings.getValue("EstimatedPath", false)) {
            for (RobotState state : this.estimatedPath) {
                if (state.pose().getTranslation().getDistance(mouseLocation) <= convertPixelsToUnits(10)) {
                    System.out.println(state);
                    break;
                }
            }
        }
    }

    @Override
    public void mousePressed(MouseEvent e, Translation2d mouseLocation) {
        System.out.println("Clicked (" + mouseLocation.getX() + "," + mouseLocation.getY() + ")");

        if (mouseLocation.getX() > this.maxValue)
            return;

        Waypoint waypoint = this.getWaypointOnMouse(mouseLocation);

        if (waypoint != null) {
            this.draggedWaypoint = waypoint;
            this.draggedObstacle = null;
        }

        boolean drag = waypoint != null;

        for (int i = this.obstacleAvoiding.getObstacles().size() - 1; i >= 0 && !drag; i--) {
            Obstacle obstacle = this.obstacleAvoiding.getObstacles().get(i);
            if (obstacle instanceof DraggableObstacle) {
                if (obstacle.getCenter().getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                    this.draggedWaypoint = null;
                    this.draggedObstacle = obstacle;
                    drag = true;
                }
            }
        }
    }

    @Override
    public void mouseReleased(MouseEvent e, Translation2d mouseLocation) {
        System.out.println("Released (" + convertPixelsToX(e.getX(), FIELD_DIMENSION) + "," + convertPixelsToY(e.getY(), FIELD_DIMENSION) + ")");
        this.draggedWaypoint = null;
        this.draggedObstacle = null;
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e, Translation2d mouseLocation) {
        if (this.selectedWaypoint != null) {
            Rotation2d heading = Rotation2d.fromDegrees(this.selectedWaypoint.getHeading().getDegrees() + (e.getPreciseWheelRotation() * 5));
            this.selectedWaypoint.setHeading(heading);
            this.waypointSettings.setValue("Heading", this.selectedWaypoint.getHeading().getDegrees() + (e.getPreciseWheelRotation() * 5));
        } else {
            this.maxValue += e.getPreciseWheelRotation() / 2;
        }
    }

    private Waypoint getWaypointOnMouse(Translation2d mouseLocation) {
        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0; i--) {
            Waypoint waypoint = this.purePursuit.getWaypoints().get(i);
            if (this.purePursuit.getWaypoints().get(i).getTranslation2d().getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                return waypoint;
            }
        }
        return null;
    }

    private void drawWaypoint(Translation2d waypoint) {
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(7), Color.WHITE);
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(6), Color.BLUE);
    }

    private void drawSelectedWaypoint(Translation2d waypoint) {
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(8), Color.WHITE);
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(6), Color.GREEN);
    }

    private static Translation2d[] getModulesLocation() {
        return new Translation2d[]{
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };
    }
}
