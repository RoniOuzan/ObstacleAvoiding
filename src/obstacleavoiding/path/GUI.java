package obstacleavoiding.path;

import obstacleavoiding.gui.Frame;
import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroLeftBottom;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.*;
import obstacleavoiding.path.obstacles.DraggableObstacle;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.*;

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

    private static final double DEFAULT_MAX_VALUE = 16.54; // 8.27
    private static final Dimension2d DIMENSION = new Dimension2d(1713, 837);
    private static final double DEFAULT_MAX_Y = DEFAULT_MAX_VALUE * ((double) DIMENSION.getY() / DIMENSION.getX());
    private static final double PIXELS_IN_ONE_UNIT = convertMaxValueToPixels(DEFAULT_MAX_VALUE);

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.75;
    private static final double BUMPER_WIDTH = 0.08;
    public static final double ROBOT_WITH_BUMPER = BUMPER_WIDTH + ROBOT_WIDTH + BUMPER_WIDTH;
    public static final double HALF_ROBOT = ROBOT_WITH_BUMPER / 2;

    private static final double MINI_ROBOT_WIDTH = ROBOT_WITH_BUMPER;

    private final List<Obstacle> obstacles;
    private final ObstacleAvoiding obstacleAvoiding;
    private final List<Waypoint> defaultWaypoints;
    private final PurePursuit purePursuit;
    private final Robot robot;

    private final Image fieldImage = new ImageIcon("src/obstacleavoiding/path/images/Field.png").getImage();
    private final Image robotImage = new ImageIcon("src/obstacleavoiding/path/images/Robot.png").getImage();
    private final Image invisibleRobotImage = new ImageIcon("src/obstacleavoiding/path/images/InvisibleRobot.png").getImage();

    private boolean showAlienatedObstacles = false;
    private boolean autoGeneratePath = true;

    private final Map<Pose2d, Integer> positions = new HashMap<>();

    private final List<Double> currentWaypoint = new ArrayList<>();
    private final List<Double> driveVelocities = new ArrayList<>();
    private final List<Double> targetDriveVelocities = new ArrayList<>();
    private final List<Double> driftPercentageVelocities = new ArrayList<>();
    private final List<Double> omegaVelocities = new ArrayList<>();
    private final List<Double> drivingAngles = new ArrayList<>();
    private final List<Double> robotAngle = new ArrayList<>();
    private final List<Double> distance = new ArrayList<>();

    private final Set<Integer> pressedKeys = new HashSet<>();

    private double maxValue = DEFAULT_MAX_VALUE;

    private Waypoint draggedWaypoint = null;
    private Obstacle draggedObstacle = null;

    public GUI() {
        super("Path Follower", DIMENSION, PIXELS_IN_ONE_UNIT);

        this.robot = new Robot(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new Robot.Constants(4.5, 360, 1 / FPS));

        List<Obstacle> blueObstacles = Arrays.asList(
                new Obstacle("BlueRamp", Alliance.BLUE,
                        new Translation2d(2.951, 3.985),
                        new Translation2d(4.828, 3.985),
                        new Translation2d(4.828, 1.509),
                        new Translation2d(2.951, 1.509)),
                new Obstacle("BlueBarrier", Alliance.BLUE ,
                        new Translation2d(1.458, 5.506),
                        new Translation2d(3.363, 5.506),
                        new Translation2d(3.363, 5.474),
                        new Translation2d(1.458, 5.474)),
                new Obstacle("BlueGrid", Alliance.BLUE,
                        new Translation2d(0, 5.500),
                        new Translation2d(1.381, 5.500),
                        new Translation2d(1.381, 0),
                        new Translation2d(0, 0)));
        List<Obstacle> redObstacles = blueObstacles.stream().map(o ->
                        new Obstacle(
                                o.getName(),
                                Alliance.RED,
                                o.getCorners().stream().map(c -> new Translation2d(DEFAULT_MAX_VALUE - c.getX(), c.getY())).toList()))
                .toList();

        this.obstacles = new ArrayList<>(Arrays.asList(
                new Obstacle("RightX", Alliance.NONE,
                        new Translation2d(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y),
                        new Translation2d(DEFAULT_MAX_VALUE + 0.1, DEFAULT_MAX_Y),
                        new Translation2d(DEFAULT_MAX_VALUE + 0.1, 0),
                        new Translation2d(DEFAULT_MAX_VALUE, 0)),
                new Obstacle("LeftX", Alliance.NONE,
                        new Translation2d(0, DEFAULT_MAX_Y),
                        new Translation2d(-0.1, DEFAULT_MAX_Y),
                        new Translation2d(-0.1, -DEFAULT_MAX_Y),
                        new Translation2d(0, -DEFAULT_MAX_Y)),
                new Obstacle("UpY", Alliance.NONE,
                        new Translation2d(0, DEFAULT_MAX_Y + 0.1),
                        new Translation2d(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y + 0.1),
                        new Translation2d(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y),
                        new Translation2d(0, DEFAULT_MAX_Y)),
                new Obstacle("DownY", Alliance.NONE,
                        new Translation2d(0, -0.1),
                        new Translation2d(DEFAULT_MAX_VALUE, -0.1),
                        new Translation2d(DEFAULT_MAX_VALUE, 0),
                        new Translation2d(0, 0)),

                new DraggableObstacle("Robot", Alliance.RED,
                        new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2).plus(new Translation2d(Math.hypot(HALF_ROBOT, HALF_ROBOT), Rotation2d.fromDegrees(45))),
                        new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2).plus(new Translation2d(Math.hypot(HALF_ROBOT, HALF_ROBOT), Rotation2d.fromDegrees(135))),
                        new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2).plus(new Translation2d(Math.hypot(HALF_ROBOT, HALF_ROBOT), Rotation2d.fromDegrees(-135))),
                        new Translation2d(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2).plus(new Translation2d(Math.hypot(HALF_ROBOT, HALF_ROBOT), Rotation2d.fromDegrees(-45))))
        ));
        this.obstacles.addAll(blueObstacles);
        this.obstacles.addAll(redObstacles);

        this.obstacleAvoiding = new ObstacleAvoiding(HALF_ROBOT, new Bounds(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y), this.obstacles);

        this.defaultWaypoints = new ArrayList<>();
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 2, DEFAULT_MAX_Y / 2 + 2, 90, Waypoint.RobotReference.CENTER));
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 - 2, DEFAULT_MAX_Y / 2 - 2, 0, Waypoint.RobotReference.BACK_CENTER));

        this.purePursuit = new PurePursuit(
                this.robot,
                new PurePursuit.Constants(1, 4, 1.5),
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
            this.drawImage(fieldImage, convertX(0, DIMENSION), (int) (convertY(0, DIMENSION) - convertUnits(DEFAULT_MAX_Y)),
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
                    MINI_ROBOT_WIDTH, MINI_ROBOT_WIDTH,
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
        }
    }

    private void manualDrive() {
        if (this.purePursuit.isRunning())
            return;

        double velocity = this.robot.getConstants().maxVel();
        double omega = this.robot.getConstants().maxOmegaVel();
        Pose2d vector = new Pose2d();
        if (this.pressedKeys.contains(KeyEvent.VK_W))
            vector = vector.plus(new Transform2d(new Translation2d(0, velocity), new Rotation2d()));
        if (this.pressedKeys.contains(KeyEvent.VK_S))
            vector = vector.plus(new Transform2d(new Translation2d(0, -velocity), new Rotation2d()));
        if (this.pressedKeys.contains(KeyEvent.VK_A))
            vector = vector.plus(new Transform2d(new Translation2d(-velocity, 0), new Rotation2d()));
        if (this.pressedKeys.contains(KeyEvent.VK_D))
            vector = vector.plus(new Transform2d(new Translation2d(velocity, 0), new Rotation2d()));
        if (this.pressedKeys.contains(KeyEvent.VK_E))
            vector = new Pose2d(vector.getTranslation(), vector.getRotation().rotateBy(Rotation2d.fromRadians(-omega)));
        if (this.pressedKeys.contains(KeyEvent.VK_Q))
            vector = new Pose2d(vector.getTranslation(), vector.getRotation().rotateBy(Rotation2d.fromRadians(omega)));

        this.robot.drive(vector);
    }

    public void start() {
        long lastUpdate = System.currentTimeMillis();

        while (true) {
            this.setPixelsInOneUnit(convertMaxValueToPixels(this.maxValue));

            this.clearFrame();
            this.drawBackground();
            this.purePursuit.update(4.5, 3, 360, 360);
            this.updateValues();
            this.displayPath();
            this.displayRobot();
            this.displayObstacles();
            this.writeValues();
            this.manualDrive();

            this.repaint();

            try {
                long period = System.currentTimeMillis() - lastUpdate;
                lastUpdate = System.currentTimeMillis();
                Thread.sleep((long) Math.max(((1000 - period) / FPS), 0));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
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
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_R) {
            this.purePursuit.reset();
            this.positions.clear();
            this.currentWaypoint.clear();
            this.driveVelocities.clear();
            this.targetDriveVelocities.clear();
            this.driftPercentageVelocities.clear();
            this.omegaVelocities.clear();
            this.drivingAngles.clear();
            this.robotAngle.clear();
            this.distance.clear();

            this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
        } else if (e.getKeyCode() == KeyEvent.VK_T) {
            this.purePursuit.setRunning(!this.purePursuit.isRunning());
        } else if (e.getKeyCode() == KeyEvent.VK_G) {
            this.showAlienatedObstacles = !this.showAlienatedObstacles;
        } else if (e.getKeyCode() == KeyEvent.VK_A) {
            this.autoGeneratePath = !this.autoGeneratePath;
        } else if (e.getKeyCode() == KeyEvent.VK_F) {
            this.obstacleAvoiding.setFiltering(!this.obstacleAvoiding.isFiltering());
        }

        System.out.println("added " + e.getKeyChar());
        this.pressedKeys.add(e.getKeyCode());
    }

    @Override
    public void keyReleased(KeyEvent e) {
        System.out.println("removed " + e.getKeyChar());
        this.pressedKeys.remove((Integer) e.getKeyCode());
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
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        System.out.println("Released (" + convertPixelsToX(e.getX(), DIMENSION) + "," + convertPixelsToY(e.getY(), DIMENSION) + ")");
        this.draggedWaypoint = null;
        this.draggedObstacle = null;
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        this.maxValue += e.getPreciseWheelRotation();
    }

    private void drawRobotPose(Translation2d pose) {
        this.fillPoint(pose.getX(), pose.getY(), convertPixelsToUnits(2), Color.RED);
        this.drawPoint(pose.getX(), pose.getY(), convertPixelsToUnits(2), new Color(20, 20, 20));
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

    private static double convertMaxValueToPixels(double maxValue) {
        return (DIMENSION.getX() / maxValue);
    }
}
