package obstacleavoiding.path;

import obstacleavoiding.gui.Frame;
import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroLeftBottom;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDPreset;
import obstacleavoiding.path.util.Alliance;
import obstacleavoiding.path.util.Bounds;
import obstacleavoiding.path.util.Obstacle;
import obstacleavoiding.path.util.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    private final Image fieldImage = new ImageIcon("src/obstacleavoiding/path/util/Field.png").getImage();
    private final Image robotImage = new ImageIcon("src/obstacleavoiding/path/util/Robot.png").getImage();
    private final Image invisibleRobotImage = new ImageIcon("src/obstacleavoiding/path/util/InvisibleRobot.png").getImage();

    private boolean showAlienatedObstacles = false;

    private final List<Pose2d> positions = new ArrayList<>();

    private final List<Double> driveVelocities = new ArrayList<>();
    private final List<Double> driftPercentageVelocities = new ArrayList<>();
    private final List<Double> omegaVelocities = new ArrayList<>();
    private final List<Double> drivingAngles = new ArrayList<>();

    private double maxValue = DEFAULT_MAX_VALUE;

    public GUI() {
        super("Path Follower", DIMENSION, PIXELS_IN_ONE_UNIT);

        this.robot = new Robot(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new Robot.Constants(5, 1 / FPS));

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
                        new Translation2d(0, 0))
        ));
        this.obstacles.addAll(blueObstacles);
        this.obstacles.addAll(redObstacles);

        this.obstacleAvoiding = new ObstacleAvoiding(HALF_ROBOT + 0.05, new Bounds(DEFAULT_MAX_VALUE, DEFAULT_MAX_Y), this.obstacles);

        this.defaultWaypoints = new ArrayList<>();
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2, DEFAULT_MAX_Y / 2, Waypoint.RobotReference.CENTER));
        this.defaultWaypoints.add(new Waypoint(DEFAULT_MAX_VALUE / 2 + 0.5, DEFAULT_MAX_Y / 2 + 0.5, Waypoint.RobotReference.BACK_CENTER));

        this.purePursuit = new PurePursuit(
                this.robot,
                new PurePursuit.Constants(4.5, 4.5, 2, 3,
                        new PIDPreset(2, 0, 0, 80, 60),
                        new PIDPreset(4.69, 0, 0, 1080, 1440)
                ),
                this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints)
        );

        this.addGraph("Drive Velocity", () -> driveVelocities, 0, this.purePursuit.getConstants().maxVel());
        this.addGraph("Last Drift", () -> driftPercentageVelocities, 0, 1);
//        this.addGraph("Angle Velocity", () -> omegaVelocities, -360, 360);
        this.addGraph("Driving Angle", () -> drivingAngles, -180, 180);

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
            this.drawWaypoint(waypoint);

            this.drawImage(invisibleRobotImage,
                    waypoint,
                    ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                    waypoint.getHeading());
        }
        this.drawConnectedPoints(Color.BLACK, this.purePursuit.getWaypoints());
    }

    public void displayPath() {
        for (Pose2d position : this.positions) {
            this.drawImage(invisibleRobotImage,
                    position.getTranslation(),
                    ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                    -position.getRotation().getDegrees());
        }
    }

    public void displayRobot() {
        if (this.purePursuit.isRunning()) {
            Translation2d driveSetpoint = this.robot.getPosition().getTranslation()
                    .plus(new Translation2d(
                            this.purePursuit.getDriveController().getSetpoint().position,
                            this.robot.getVelocity().getTranslation().getAngle()));
            this.fillPoint(driveSetpoint.getX(), driveSetpoint.getY(), convertPixelsToUnits(5), Color.GREEN);
        }

        Translation2d omegaSetpoint = this.robot.getPosition().getTranslation()
                .plus(new Translation2d(2 * ROBOT_WITH_BUMPER,
                        Rotation2d.fromDegrees(this.purePursuit.getOmegaController().getSetpoint().position)));
        this.fillPoint(omegaSetpoint, convertPixelsToUnits(5), Color.GREEN);

        this.drawImage(robotImage,
                this.robot.getPosition().getTranslation(),
                ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                -this.robot.getPosition().getRotation().getDegrees());
    }

    public void displayObstacles() {
        for (Obstacle obstacle : this.obstacles) {
            this.fillPolygon(obstacle.getAlliance().getColor(50), obstacle.getCorners());
        }
        if (this.showAlienatedObstacles) {
            for (Obstacle obstacle : this.obstacleAvoiding.getObstacles()) {
                this.fillPolygon(new Color(0, 0, 0, 30), obstacle.getCorners());
            }
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
        if (this.purePursuit.isRunning() && !this.purePursuit.isFinished()) {
            this.positions.add(this.robot.getPosition());

            this.driveVelocities.add(this.robot.getVelocity().getTranslation().getNorm());
            this.driftPercentageVelocities.add(this.purePursuit.getDriftPercentage());
            this.omegaVelocities.add(this.robot.getVelocity().getRotation().getDegrees());
            this.drivingAngles.add(this.robot.getVelocity().getTranslation().getAngle().getDegrees());
        }
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        Translation2d mouseLocation = this.getMouseTranslation(e);

        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0; i--) {
            if (this.purePursuit.getWaypoints().get(i).getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                Waypoint lastWaypoint = this.purePursuit.getWaypoints().get(i);
                Waypoint waypoint = new Waypoint(mouseLocation.minus(lastWaypoint.getRobotReferenceTranslation()), lastWaypoint);
                this.purePursuit.getWaypoints().set(i, waypoint);

                for (int j = 0; j < this.defaultWaypoints.size(); j++) {
                    if (this.defaultWaypoints.get(j) == lastWaypoint) {
                        this.defaultWaypoints.set(j, waypoint);
                    }
                }

                break;
            }
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyChar() == 'r' || e.getKeyChar() == 'R' || e.getKeyChar() == 'ר') {
            this.purePursuit.reset();
            this.positions.clear();
            this.driveVelocities.clear();
            this.driftPercentageVelocities.clear();
            this.omegaVelocities.clear();
            this.drivingAngles.clear();
            this.purePursuit.setWaypoints(this.obstacleAvoiding.generateWaypointsBinary(this.defaultWaypoints));
        } else if (e.getKeyChar() == 't' || e.getKeyChar() == 'T' || e.getKeyChar() == 'א') {
            this.purePursuit.setRunning(!this.purePursuit.isRunning());
        } else if (e.getKeyChar() == 'g' || e.getKeyChar() == 'G' || e.getKeyChar() == 'ע') {
            this.showAlienatedObstacles = !this.showAlienatedObstacles;
        } else if (e.getKeyChar() == 'f' || e.getKeyChar() == 'F' || e.getKeyChar() == 'כ') {
            this.obstacleAvoiding.setFiltering(!this.obstacleAvoiding.isFiltering());
        }

    }

    @Override
    public void mousePressed(MouseEvent e) {
        System.out.println("Clicked (" + convertPixelsToX(e.getX(), DIMENSION) + "," + convertPixelsToY(e.getY(), DIMENSION) + ")");
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        System.out.println("Released (" + convertPixelsToX(e.getX(), DIMENSION) + "," + convertPixelsToY(e.getY(), DIMENSION) + ")");
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        this.maxValue += e.getPreciseWheelRotation();
    }

    public void start() {
        while (true) {
            this.setPixelsInOneUnit(convertMaxValueToPixels(this.maxValue));

            this.clearFrame();
            this.drawBackground();
            this.purePursuit.update();
            this.updateValues();
            this.displayPath();
            this.displayRobot();
            this.displayObstacles();
            this.writeValues();
            this.repaint();

            try {
                Thread.sleep((long) (1000 / FPS));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
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
