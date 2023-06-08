package obstacleavoiding.path;

import com.github.strikerx3.jxinput.XInputComponents;
import com.github.strikerx3.jxinput.XInputDevice;
import obstacleavoiding.gui.Frame;
import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroLeftBottom;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.*;
import obstacleavoiding.path.fields.ChargedUpField;
import obstacleavoiding.path.fields.Field;
import obstacleavoiding.path.obstacles.DraggableObstacle;
import obstacleavoiding.path.obstacles.Obstacle;
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
    private static final boolean IS_CHARGED_UP_FIELD = true;

    private static final int SETTINGS_WIDTH = 150;

    private static final double DEFAULT_MAX_VALUE = 16.54; // 8.27
    private static final Dimension2d FIELD_DIMENSION = new Dimension2d(1713, 837);
    private static final double DEFAULT_MAX_Y = DEFAULT_MAX_VALUE * ((double) FIELD_DIMENSION.getY() / FIELD_DIMENSION.getX());

    private static final int GRAPH_HISTORY = 50;

    private static final double FPS = 20;
    private static final double PERIOD = 1 / FPS;
    private static final double ROBOT_WIDTH = 0.75;
    private static final double BUMPER_WIDTH = 0.08;
    public static final double ROBOT_WITH_BUMPER = BUMPER_WIDTH + ROBOT_WIDTH + BUMPER_WIDTH;
    public static final double HALF_ROBOT = ROBOT_WITH_BUMPER / 2;

    private static final double MINI_ROBOT_WITH_BUMPER = ROBOT_WITH_BUMPER;

    private final Field field;

    private final List<Obstacle> obstacles;
    private final ObstacleAvoiding obstacleAvoiding;
    private final List<Waypoint> defaultWaypoints;
    private final PurePursuit purePursuit;
    private final Robot robot;

    private final Image robotImage = new ImageIcon("src/main/java/obstacleavoiding/path/images/Robot.png").getImage();
    private final Image invisibleRobotImage = new ImageIcon("src/main/java/obstacleavoiding/path/images/InvisibleRobot.png").getImage();

    private boolean showAlienatedObstacles = false;
    private boolean autoGeneratePath = false;

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

    public GUI() {
        super("Path Follower", FIELD_DIMENSION.plus(new Dimension2d(SETTINGS_WIDTH, 0)), FIELD_DIMENSION.getX() / DEFAULT_MAX_VALUE, FPS);
        this.addDevice(0);

        this.field = new ChargedUpField();
        this.obstacles = new ArrayList<>(this.field.getObstacles());

        this.robot = new Robot(new Pose2d(new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2), Rotation2d.fromDegrees(0)),
                new Robot.Constants(4.5, 9, 360, 720, 1 / FPS));

        this.obstacleAvoiding = new ObstacleAvoiding(0.5, new Bounds(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y), this.obstacles);

        this.defaultWaypoints = new ArrayList<>();
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 2, DEFAULT_MAX_Y / 2 + 2, 90, Waypoint.RobotReference.CENTER));
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

        this.purePursuit.reset();
        this.start();
    }

    public void drawBackground() {
        if (IS_CHARGED_UP_FIELD)
            this.drawImage(this.field.getImage(), convertX(0, this.getDimension()), (int) (convertY(0, this.getDimension()) - convertUnits(DEFAULT_MAX_Y)),
                    (int) convertUnits(DEFAULT_MAX_VALUE), (int) convertUnits(DEFAULT_MAX_Y));
        else
            this.drawGrid();

        for (Waypoint waypoint : this.purePursuit.getWaypoints()) {
            this.drawWaypoint(waypoint.getOriginalPosition());

            this.drawImage(invisibleRobotImage,
                    waypoint,
                    ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                    -waypoint.getHeading());
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
        for (Obstacle obstacle : this.obstacles) {
            this.fillPolygon(obstacle.getAlliance().getColor(50), obstacle.getCorners());
            if (obstacle instanceof DraggableObstacle) {
                this.fillPoint(obstacle.getCenter(), convertPixelsToUnits(7), Color.RED);
            }
        }
        if (this.showAlienatedObstacles) {
            this.obstacles.stream().map(o -> o.getExtendedObstacle(this.obstacleAvoiding.getDistanceThreshold()))
                    .forEach(o -> this.fillPolygon(new Color(0, 0, 0, 30), o.getCorners()));
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
    }

    private void limitList(List<?> list) {
        while (list.size() > GRAPH_HISTORY)
            list.remove(0);
    }

    public void reset() {
        this.purePursuit.setRunning(false);
        this.defaultWaypoints.set(0, new Waypoint(this.robot.getPosition().getTranslation(), this.robot.getPosition().getRotation().getDegrees(), Waypoint.RobotReference.CENTER));
        this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
        this.purePursuit.setRunning(true);

        this.purePursuit.reset();
        this.robot.drive(new Pose2d());

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
        this.purePursuit.update(4.5, 3, 360, 360);
        this.updateValues();
        this.displayPath();
        this.displayObstacles();
        this.writeValues();
        this.displayRobot();
    }

    @Override
    public double getPixelsInUnits() {
        return this.getDimension().getX() / this.maxValue;
    }

    @Override
        public void deviceListen(XInputDevice device, XInputComponents components, XInputComponents lastComponents) {
        if (components.getButtons().a && !lastComponents.getButtons().a) {
            this.reset();
        } else if (components.getButtons().b && !lastComponents.getButtons().b) {
            this.purePursuit.setRunning(!this.purePursuit.isRunning());
        }

        if (this.purePursuit.isRunning())
            return;

        this.isControllerDrive = true;

        Translation2d leftAxis = new Translation2d(
                MathUtil.limitDot(components.getAxes().lx, 3),
                MathUtil.limitDot(components.getAxes().ly, 3)
        ).normalized();
        double rightAxis = -Math.pow(MathUtil.limitDot(components.getAxes().rx, 3), 1) * this.robot.getConstants().maxOmegaVel();

        Rotation2d angle = leftAxis.getNorm() < 0.01 ? this.robot.getVelocity().getTranslation().getAngle() : leftAxis.getAngle();
        double magnitude = Math.pow(leftAxis.getNorm(), 2) * this.robot.getConstants().maxVel();

        Translation2d accel = new Translation2d(magnitude, angle).minus(this.robot.getVelocity().getTranslation());
        accel = new Translation2d(Math.min(accel.getNorm(), this.robot.getConstants().maxAccel() * PERIOD), accel.getAngle());

        double omegaAccel = MathUtil.clamp(rightAxis - this.robot.getVelocity().getRotation().getDegrees(),
                -this.robot.getConstants().maxOmegaAccel() * PERIOD, this.robot.getConstants().maxOmegaAccel() * PERIOD);

        this.robot.drive(new Pose2d(
                this.robot.getVelocity().getTranslation().plus(accel),
                 Rotation2d.fromDegrees(this.robot.getVelocity().getRotation().getDegrees() + omegaAccel)));
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        Translation2d mouseLocation = this.getMouseTranslation(e);

        boolean drag = false;
        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0 && !drag; i--) {
            Waypoint waypoint = this.purePursuit.getWaypoints().get(i);
            if (waypoint == this.draggedWaypoint) {
                waypoint.set(mouseLocation);

                if (this.autoGeneratePath) {
                    this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
                }

                drag = true;
            }
        }

        for (int i = this.obstacles.size() - 1; i >= 0 && !drag; i--) {
            Obstacle obstacle = this.obstacles.get(i);
            if (obstacle == this.draggedObstacle) {
                obstacle.setCorners(DraggableObstacle.getCornersOfObstacle(mouseLocation, obstacle));

                if (this.autoGeneratePath) {
                    this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
                }

                drag = true;
            }
        }
    }

    @Override
    public void keyListen(Set<Integer> keys) {
        if (this.purePursuit.isRunning() || !this.isControllerDrive || keys.size() == 0)
            return;

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

        if (velocity.getTranslation().getNorm() >= 0.1 || velocity.getRotation().getDegrees() > 1)
            this.isControllerDrive = false;

        this.robot.drive(new Pose2d(
                velocity.getTranslation().normalized().times(this.robot.getConstants().maxVel() * 0.6),
                velocity.getRotation()));
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_R) {
            this.reset();
        } else if (e.getKeyCode() == KeyEvent.VK_T) {
            this.purePursuit.setRunning(!this.purePursuit.isRunning());
        } else if (e.getKeyCode() == KeyEvent.VK_G) {
            this.showAlienatedObstacles = !this.showAlienatedObstacles;
        } else if (e.getKeyCode() == KeyEvent.VK_Y) {
            this.autoGeneratePath = !this.autoGeneratePath;
        } else if (e.getKeyCode() == KeyEvent.VK_F) {
            this.obstacleAvoiding.setFiltering(!this.obstacleAvoiding.isFiltering());
        }
    }

    @Override
    public void mousePressed(MouseEvent e) {
        Translation2d mouseLocation = this.getMouseTranslation(e);
        System.out.println("Clicked (" + mouseLocation.getX() + "," + mouseLocation.getY() + ")");

        boolean drag = false;
        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0 && !drag; i--) {
            Waypoint waypoint = this.purePursuit.getWaypoint(i);
            if (this.purePursuit.getWaypoints().get(i).getOriginalPosition().getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                this.draggedWaypoint = waypoint;
                this.draggedObstacle = null;
                drag = true;
            }
        }

        for (int i = this.obstacles.size() - 1; i >= 0 && !drag; i--) {
            Obstacle obstacle = this.obstacles.get(i);
            if (obstacle instanceof DraggableObstacle) {
                if (obstacle.getCenter().getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                    this.draggedWaypoint = null;
                    this.draggedObstacle = obstacle;
                    drag = true;
                }
            }
        }

        if (!drag) {
            this.robot.drive(new Pose2d());

            this.defaultWaypoints.get(0).set(this.robot.getPosition());
            this.defaultWaypoints.get(this.defaultWaypoints.size() - 1).set(mouseLocation);

            this.reset();
        }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        System.out.println("Released (" + convertPixelsToX(e.getX(), FIELD_DIMENSION) + "," + convertPixelsToY(e.getY(), FIELD_DIMENSION) + ")");
        this.draggedWaypoint = null;
        this.draggedObstacle = null;
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        this.maxValue += e.getPreciseWheelRotation();
    }

    private void drawWaypoint(Translation2d waypoint) {
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(6), Color.BLUE);
        this.drawPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(7), new Color(200, 200, 200));
    }

    private void drawGrid() {
        for (double i = (int) -Math.floor(this.maxValue); i <= this.maxValue; i += 0.25) {
            this.drawThinLine(i, this.getDimensionWithUnits().getY() / -2, i, this.getDimensionWithUnits().getY() / 2,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        for (double i = (int) -Math.floor(this.maxValue); i <= this.maxValue; i += 0.25) {
            this.drawThinLine(this.getDimensionWithUnits().getX() / -2, i, this.getDimensionWithUnits().getX() / 2, i,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        this.drawLine(0, this.getDimensionWithUnits().getY() / -2, 0, this.getDimensionWithUnits().getY() / 2, convertPixelsToUnits(5), Color.BLACK);
        this.drawLine(this.getDimensionWithUnits().getX() / -2, 0, this.getDimensionWithUnits().getX() / 2, 0, convertPixelsToUnits(5), Color.BLACK);

        double textSize = convertPixelsToUnits(20);
        for (int i = 0; i <= this.maxValue; i += this.maxValue / 10) {
            this.write(0, i - (textSize / 2), " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(0, -i - (textSize / 2), -i + "", textSize, Color.BLACK);
        }
        for (int i = 0; i <= this.maxValue; i += this.maxValue / 10 ) {
            this.write(i - (textSize / 2), -textSize, " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(-i - (textSize / 2), -textSize, -i + "", textSize, Color.BLACK);
        }
    }
}
