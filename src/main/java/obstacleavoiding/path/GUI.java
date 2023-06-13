package obstacleavoiding.path;

import com.github.strikerx3.jxinput.XInputComponents;
import com.github.strikerx3.jxinput.XInputDevice;
import obstacleavoiding.gui.Frame;
import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroLeftBottom;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.*;
import obstacleavoiding.path.fields.Fields;
import obstacleavoiding.path.obstacles.DraggableObstacle;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.settings.WaypointSettings;
import obstacleavoiding.path.settings.tables.BooleanTable;
import obstacleavoiding.path.settings.tables.SelectOptionTable;
import obstacleavoiding.path.settings.tables.SliderTable;
import obstacleavoiding.path.settings.tables.TableType;
import obstacleavoiding.path.util.Alliance;
import obstacleavoiding.path.util.Bounds;
import obstacleavoiding.path.util.Waypoint;

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
    public static final Dimension2d FIELD_DIMENSION = new Dimension2d(1713, 837);
    public static final Dimension2d FRAME_DIMENSION = FIELD_DIMENSION.plus(new Dimension2d(SETTINGS_WIDTH, 0));
    public static final double DEFAULT_MAX_Y = DEFAULT_MAX_VALUE * ((double) FIELD_DIMENSION.getY() / FIELD_DIMENSION.getX());

    private static final int GRAPH_HISTORY = 50;

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.75;
    private static final double BUMPER_WIDTH = 0.08;
    public static final double ROBOT_WITH_BUMPER = BUMPER_WIDTH + ROBOT_WIDTH + BUMPER_WIDTH;
    public static final double HALF_ROBOT = ROBOT_WITH_BUMPER / 2;

    private static final double MINI_ROBOT_WITH_BUMPER = ROBOT_WITH_BUMPER;

    private static final Fields DEFAULT_FIELD = Fields.CHARGED_UP;
    private static final Alliance DEFAULT_ALLIANCE = Alliance.NONE;

    private static final String IMAGES_PATH = "src/main/java/obstacleavoiding/path/images/";

    private final Settings settings;
    private final WaypointSettings waypointSettings;

    private final ObstacleAvoiding obstacleAvoiding;
    private final List<Waypoint> defaultWaypoints;
    private final PurePursuit purePursuit;
    private final Robot robot;

    private Image robotImage = new ImageIcon(IMAGES_PATH + DEFAULT_ALLIANCE.getText() + "Robot.png").getImage();
    private Image invisibleRobotImage = new ImageIcon(IMAGES_PATH + DEFAULT_ALLIANCE.getText() + "InvisibleRobot.png").getImage();

    private boolean isControllerDrive = true;

    private final Map<Pose2d, Integer> positions = new HashMap<>();

    private final List<Double> currentWaypoint = new ArrayList<>();
    private final List<Double> driveVelocities = new ArrayList<>();
    private final List<Double> targetDriveVelocities = new ArrayList<>();
    private final List<Double> driftPercentageVelocities = new ArrayList<>();
    private final List<Double> omegaVelocities = new ArrayList<>();
    private final List<Double> drivingAngles = new ArrayList<>();
    private final List<Double> robotAngle = new ArrayList<>();
    private final List<Double> distance = new ArrayList<>();

    private double maxValue = DEFAULT_MAX_VALUE;

    private Waypoint draggedWaypoint = null;
    private Obstacle draggedObstacle = null;

    private Waypoint selectedWaypoint = null;

    public GUI() {
        super("Path Follower", FRAME_DIMENSION, FPS);
        this.addDevice(0);

        this.robot = new Robot(new Pose2d(new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2), Rotation2d.fromDegrees(0)),
                new Robot.Constants(4.5, 9, 360, 720, 1 / FPS));

        this.obstacleAvoiding = new ObstacleAvoiding(HALF_ROBOT + 0.05, new Bounds(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y), DEFAULT_FIELD.getField().getObstacles());

        this.defaultWaypoints = new ArrayList<>();
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 2, DEFAULT_MAX_Y / 2 + 2, 90, Waypoint.RobotReference.CENTER));
//        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 1, DEFAULT_MAX_Y / 2 + 1, 90, Waypoint.RobotReference.CENTER));
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 - 2, DEFAULT_MAX_Y / 2 - 2, 0, Waypoint.RobotReference.CENTER));

        this.purePursuit = new PurePursuit(
                this.robot,
                new PurePursuit.Constants(1, 4, 3),
                this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints)
        );

        Map<Supplier<List<Double>>, Color> graphValues = new HashMap<>();
        graphValues.put(() -> driftPercentageVelocities, new Color(0, 0, 255));
        graphValues.put(() -> currentWaypoint, new Color(0, 255, 0));
        graphValues.put(() -> driveVelocities.stream().map(s -> s / this.robot.getConstants().maxVel()).toList(), new Color(255, 0, 0));
        graphValues.put(() -> targetDriveVelocities.stream().map(s -> s / this.robot.getConstants().maxVel()).toList(), new Color(100, 0, 0));
        graphValues.put(() -> omegaVelocities.stream().map(s -> (s + 180) / this.robot.getConstants().maxOmegaVel()).toList(), new Color(255, 255, 0));
        graphValues.put(() -> drivingAngles.stream().map(s -> MathUtil.inputModulus(s / 360 + 0.5, 0, 1)).toList(), new Color(0, 255, 255));
        graphValues.put(() -> robotAngle.stream().map(s -> MathUtil.inputModulus(s / 360 + 0.5, 0, 1)).toList(), new Color(135, 206, 235));
        graphValues.put(() -> distance.stream().map(d -> d / this.purePursuit.getPathDistance()).toList(), new Color(244, 123, 156));

        this.addGraph("Values", graphValues, 0, 1);

        List<TableType<?>> values = new ArrayList<>();
        values.add(new SelectOptionTable<>("Field", DEFAULT_FIELD, Fields.values()).onChange((c, l) -> {
            this.obstacleAvoiding.setObstacles(c.getField().getObstacles());
            this.reset();
        }));
        values.add(new SelectOptionTable<>("Alliance", DEFAULT_ALLIANCE, Alliance.values()).onChange((c, l) -> {
            robotImage = new ImageIcon(IMAGES_PATH + c.getText() + "Robot.png").getImage();
            invisibleRobotImage = new ImageIcon(IMAGES_PATH + c.getText() + "InvisibleRobot.png").getImage();
        }));
        values.add(new SliderTable("FPS", FPS, 1, 50).setValueParser(this::setFPS));
        values.add(new SliderTable("MaxVel", 4.5, 0, 4.9));
        values.add(new SliderTable("MaxAccel", 8, 0, 10));
        values.add(new SliderTable("MaxOmegaVel", 360, 0, 720));
        values.add(new SliderTable("MaxOmegaAccel", 360, 0, 1080));
        values.add(new BooleanTable("Filter", true).setValueParser(this.obstacleAvoiding::setFiltering));
        values.add(new BooleanTable("Reset", false).onTrue(this::reset).setAlways(false));
        values.add(new BooleanTable("Running", true).setValueParser(this.purePursuit::setRunning));
        values.add(new BooleanTable("Extended Obstacles", false));
        values.add(new BooleanTable("Auto Generate", false).onTrue(this::resetPath));
        values.add(new BooleanTable("Create Waypoint", false).setAlways(false).onTrue(() -> {
                    this.defaultWaypoints.add(this.defaultWaypoints.size() - 1, new Waypoint(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2, Waypoint.RobotReference.CENTER));
                    this.resetPath();
                }));
        this.settings = new Settings(values);
        this.add(this.settings);

        this.waypointSettings = new WaypointSettings();
        this.add(this.waypointSettings);

        this.purePursuit.reset();
        this.start();
    }

    public void drawBackground() {
        this.drawImage(this.settings.getValue("Field", DEFAULT_FIELD).getField().getImage(), convertX(0, FIELD_DIMENSION), (int) (convertY(0, FIELD_DIMENSION) - convertUnits(DEFAULT_MAX_Y)),
                (int) convertUnits(DEFAULT_MAX_VALUE), (int) convertUnits(DEFAULT_MAX_Y));

        for (Waypoint waypoint : this.purePursuit.getWaypoints()) {
            this.drawImage(invisibleRobotImage,
                    waypoint,
                    ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                    -waypoint.getHeading());

            if (this.selectedWaypoint == waypoint) {
                this.drawSelectedWaypoint(waypoint.getOriginalPosition());
            } else {
                this.drawWaypoint(waypoint.getOriginalPosition());
            }
        }
        this.drawConnectedPoints(Color.BLACK, this.purePursuit.getWaypoints());
    }

    public void displayPath() {
        for (Map.Entry<Pose2d, Integer> entry : this.positions.entrySet()) {
            this.drawImage(invisibleRobotImage,
                    entry.getKey().getTranslation(),
                    MINI_ROBOT_WITH_BUMPER, MINI_ROBOT_WITH_BUMPER,
                    -entry.getKey().getRotation().getDegrees());
        }
    }

    public void displayRobot() {
        this.drawImage(robotImage,
                this.robot.getPosition().getTranslation(),
                ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                -this.robot.getPosition().getRotation().getDegrees());
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
                    .forEach(o -> this.fillPolygon(new Color(0, 0, 0, 30), o.getCorners()
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

        this.currentWaypoint.add(this.purePursuit.getCurrentWaypointIndex() * 1d / this.purePursuit.getWaypoints().size());
        this.driveVelocities.add(this.robot.getVelocity().getTranslation().getNorm());
        this.targetDriveVelocities.add(this.purePursuit.getTargetDriveVelocity());
        this.driftPercentageVelocities.add(this.purePursuit.getDriftPercentage());
        this.omegaVelocities.add(this.robot.getVelocity().getRotation().getDegrees());
        this.drivingAngles.add(this.robot.getVelocity().getTranslation().getAngle().getDegrees());
        this.robotAngle.add(this.robot.getPosition().getRotation().bound(0, 360).getDegrees());
        this.distance.add(this.purePursuit.getPathDistance() - this.purePursuit.getDistanceToFinalWaypoint());

        if (GRAPH_HISTORY > 0) {
            limitList(this.currentWaypoint);
            limitList(this.driveVelocities);
            limitList(this.targetDriveVelocities);
            limitList(this.driftPercentageVelocities);
            limitList(this.omegaVelocities);
            limitList(this.drivingAngles);
            limitList(this.robotAngle);
            limitList(this.distance);
        }
    }

    private void limitList(List<?> list) {
        while (list.size() > GRAPH_HISTORY)
            list.remove(0);
    }

    public void resetPath() {
        this.purePursuit.setRunning(false);
        this.defaultWaypoints.set(0, new Waypoint(this.robot.getPosition().getTranslation(), this.robot.getPosition().getRotation().getDegrees(), Waypoint.RobotReference.CENTER));
        this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
        this.purePursuit.setRunning(true);
    }

    public void reset() {
        this.resetPath();

        this.purePursuit.reset();

        this.positions.clear();

        if (GRAPH_HISTORY <= 0) {
            this.currentWaypoint.clear();
            this.driveVelocities.clear();
            this.targetDriveVelocities.clear();
            this.driftPercentageVelocities.clear();
            this.omegaVelocities.clear();
            this.drivingAngles.clear();
            this.robotAngle.clear();
            this.distance.clear();
        }

        this.purePursuit.setRunning(true);
    }

    @Override
    public void update() {
        this.drawBackground();
        this.settings.update();
        this.waypointSettings.update(this.selectedWaypoint);
        this.purePursuit.update(this.settings.getValue("MaxVel", 0d), this.settings.getValue("MaxAccel", 0d),
                this.settings.getValue("MaxOmegaVel", 0d), this.settings.getValue("MaxOmegaAccel", 0d));
        this.updateValues();
        this.displayPath();
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
                MathUtil.limitDot(components.getAxes().lx, 3),
                MathUtil.limitDot(components.getAxes().ly, 3)
        );
        double rightAxis = -Math.pow(MathUtil.limitDot(components.getAxes().rx, 3), 1) * this.robot.getConstants().maxOmegaVel();

        Rotation2d angle = leftAxis.getNorm() < 0.01 ? this.robot.getVelocity().getTranslation().getAngle() : leftAxis.getAngle();
        double magnitude = Math.pow(Math.min(leftAxis.getNorm(), 1), 2) * this.robot.getConstants().maxVel();

        Translation2d accel = new Translation2d(magnitude, angle).minus(this.robot.getVelocity().getTranslation());
        accel = new Translation2d(Math.min(accel.getNorm(), this.robot.getConstants().maxAccel() * getPeriod()), accel.getAngle());

        double omegaAccel = MathUtil.clamp(rightAxis - this.robot.getVelocity().getRotation().getDegrees(),
                -this.robot.getConstants().maxOmegaAccel() * getPeriod(), this.robot.getConstants().maxOmegaAccel() * getPeriod());

        this.robot.drive(new Pose2d(
                this.robot.getVelocity().getTranslation().plus(accel),
                 Rotation2d.fromDegrees(this.robot.getVelocity().getRotation().getDegrees() + omegaAccel)), getPeriod());

        if (this.robot.isMoving() && this.settings.getValue("Auto Generate", false)) {
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
        double omega = this.robot.getConstants().maxOmegaVel();
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
            velocity = new Pose2d(velocity.getTranslation(), velocity.getRotation().rotateBy(Rotation2d.fromRadians(-omega)));
        if (keys.contains(KeyEvent.VK_Q))
            velocity = new Pose2d(velocity.getTranslation(), velocity.getRotation().rotateBy(Rotation2d.fromRadians(omega)));

        if (velocity.getTranslation().getNorm() >= 0.01 || velocity.getRotation().getDegrees() > 0.5)
            this.isControllerDrive = false;

        if (this.purePursuit.isRunning() || this.isControllerDrive || keys.size() == 0)
            return;

        this.robot.drive(new Pose2d(
                velocity.getTranslation().normalized().times(this.robot.getConstants().maxVel() * 0.6),
                velocity.getRotation()), getPeriod());
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_R) {
            this.settings.setValue("Reset", true);
        } else if (e.getKeyCode() == KeyEvent.VK_T) {
            this.settings.setValue("Running", !this.purePursuit.isRunning());
        } else if (e.getKeyCode() == KeyEvent.VK_G) {
            this.settings.setValue("Extended Obstacles", !(boolean) this.settings.getValue("Extended Obstacles", false));
        } else if (e.getKeyCode() == KeyEvent.VK_Y) {
            this.settings.setValue("Auto Generate", !(boolean) this.settings.getValue("Auto Generate", false));
        } else if (e.getKeyCode() == KeyEvent.VK_F) {
            this.settings.setValue("Filter", !this.obstacleAvoiding.isFiltering());
        } else if (e.getKeyCode() == KeyEvent.VK_P) {
            System.out.println(this.obstacleAvoiding.getDistributingObstacles(this.purePursuit.getWaypoints()));
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
        } else {
            this.defaultWaypoints.get(this.defaultWaypoints.size() - 1).set(mouseLocation);
            this.reset();
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
            double heading = MathUtil.inputModulus(this.selectedWaypoint.getHeading() + (e.getPreciseWheelRotation() * 5), -180, 180);
            this.selectedWaypoint.setHeading(heading);
            this.waypointSettings.setValue("Heading", this.selectedWaypoint.getHeading() + (e.getPreciseWheelRotation() * 5));
        } else {
            this.maxValue += e.getPreciseWheelRotation() / 2;
        }
    }

    private Waypoint getWaypointOnMouse(Translation2d mouseLocation) {
        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0; i--) {
            Waypoint waypoint = this.purePursuit.getWaypoint(i);
            if (this.purePursuit.getWaypoints().get(i).getOriginalPosition().getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
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
}
