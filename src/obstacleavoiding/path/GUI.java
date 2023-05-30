package obstacleavoiding.path;

import obstacleavoiding.gui.types.draw.DrawCentered;
import obstacleavoiding.gui.types.field.ZeroCenter;
import obstacleavoiding.gui.Frame;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.pid.PIDPreset;
import obstacleavoiding.path.util.Obstacle;
import obstacleavoiding.path.util.Waypoint;

import java.util.List;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.ArrayList;

public class GUI extends Frame implements ZeroCenter, DrawCentered {
    private static final boolean IS_CHARGED_UP_FIELD = true;

    private static final double DEFAULT_MAX_VALUE = 8.27;
    private static final Dimension2d DIMENSION = new Dimension2d(1713, 837);
    private static final double DEFAULT_MAX_Y = DEFAULT_MAX_VALUE * ((double) DIMENSION.getY() / DIMENSION.getX());
    private static final double PIXELS_IN_ONE_UNIT = convertMaxValueToPixels(DEFAULT_MAX_VALUE);

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.75;
    private static final double HALF_ROBOT = ROBOT_WIDTH / 2;
    private static final double BUMPER_WIDTH = 0.08;
    private static final double ROBOT_WITH_BUMPER = BUMPER_WIDTH + ROBOT_WIDTH + BUMPER_WIDTH;

    private final ObstacleAvoiding obstacleAvoiding;
    private final PurePursuit purePursuit;
    private final Robot robot;

    private final List<Double> velocities = new ArrayList<>();
    private final List<Double> drivingAngles = new ArrayList<>();

    private double maxValue = DEFAULT_MAX_VALUE;

    public GUI() {
        super("Path Follower", DIMENSION, PIXELS_IN_ONE_UNIT);

        this.robot = new Robot(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new Robot.Constants(5, 1 / FPS));

        this.obstacleAvoiding = new ObstacleAvoiding(ROBOT_WITH_BUMPER / 2 + 0.1,
                new Obstacle(new Translation2d(3.422, -0.053),
                        new Translation2d(5.354, -0.053),
                        new Translation2d(5.354, -2.495),
                        new Translation2d(3.422, -2.495))
        );

        this.purePursuit = new PurePursuit(
                this.robot,
                new PurePursuit.Constants(4.5, 4.5, 15,
                        new PIDPreset(2, 0, 0, 1, 10),
                        new PIDPreset(2, 0, 0, 5, 10)
                ),
                new Waypoint(0, 0, (r, w) -> true),
                    new Waypoint(1, 1, (r, w) -> r.getPosition().getTranslation().getDistance(w) <= 0.05)
//                new Waypoint(DEFAULT_MAX_VALUE - 2.8, 4.585 - DEFAULT_MAX_Y, (r, w) -> Math.abs(r.getPosition().getX()) + 0.2 > Math.abs(w.getX()) &&
//                        MathUtil.inTolerance(r.getPosition().getY(), w.getY(), 0.5)),
//                new Waypoint(DEFAULT_MAX_VALUE - 1.9 - 0.3, 0.5112 - DEFAULT_MAX_Y, (r, w) -> MathUtil.inTolerance(r.getPosition().getTranslation().getY(), w.getY(), 0.4)),
//                new Waypoint(DEFAULT_MAX_VALUE - 1.9, 0.5112 - DEFAULT_MAX_Y, (r, w) -> r.getPosition().getTranslation().getDistance(w) <= 0.05)
        );

        this.addGraph("Velocities", () -> velocities);
        this.addGraph("Driving Angle", () -> drivingAngles);

        this.purePursuit.reset();
        this.start();
    }

    public void drawBackground() {
        if (IS_CHARGED_UP_FIELD)
            this.drawImage(new ImageIcon("src/obstacleavoiding/path/util/Field.png").getImage(), 0, 0, DIMENSION.getX(), DIMENSION.getY());
        else
            this.drawGrid();

        for (Translation2d waypoint : this.purePursuit.getWaypoints()) {
            this.drawWaypoint(waypoint);
        }
        this.drawConnectedPoints(Color.BLACK, this.purePursuit.getWaypoints());
    }

    public void displayRobot() {
        if (this.purePursuit.isRunning()) {
            Translation2d driveSetpoint = this.robot.getPosition().getTranslation()
                    .plus(new Translation2d(
                            this.purePursuit.getAbsoluteDistance() - this.purePursuit.getDriveController().getSetpoint().position,
                            this.robot.getVelocity().getTranslation().getAngle()));
            this.fillPoint(driveSetpoint.getX(), driveSetpoint.getY(), convertPixelsToUnits(5), Color.GREEN);
        }

        Translation2d omegaSetpoint = this.robot.getPosition().getTranslation()
                .plus(new Translation2d(2 * ROBOT_WITH_BUMPER,
                        Rotation2d.fromDegrees(this.purePursuit.getOmegaController().getSetpoint().position)));
        this.fillPoint(omegaSetpoint.getX(), omegaSetpoint.getY(), convertPixelsToUnits(5), Color.GREEN);

        this.drawImage(new ImageIcon("src/obstacleavoiding/path/util/Robot.png").getImage(),
                this.robot.getPosition().getX(),
                this.robot.getPosition().getY(),
                ROBOT_WITH_BUMPER, ROBOT_WITH_BUMPER,
                this.robot.getPosition().getRotation().getDegrees());
    }

    public void displayObstacles() {
        for (Obstacle obstacle : this.obstacleAvoiding.getObstacles()) {
            this.fillPolygon(new Color(255, 0, 0, 50), obstacle.getCorners());
        }
    }

    public void writeValues() {
        String[] texts = {
                "Pose: (" + MathUtil.limitDot(this.robot.getPosition().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getPosition().getTranslation().getY(), 3) + ")",
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
            this.write(-this.maxValue + convertPixelsToUnits(5), (this.maxValue * ((double) DIMENSION.getY() / DIMENSION.getX()) - ((size + space) * (i + 1))), texts[i], size, Color.BLACK);
        }
    }

    public void updateValues() {
        if (this.purePursuit.isRunning() && !this.purePursuit.isFinished()) {
            this.velocities.add(this.robot.getVelocity().getTranslation().getNorm());
            this.drivingAngles.add(this.robot.getVelocity().getTranslation().getAngle().getDegrees());
        }
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        Translation2d mouseLocation = this.getMouseTranslation(e);

        for (int i = this.purePursuit.getWaypoints().size() - 1; i >= 0; i--) {
            if (this.purePursuit.getWaypoints().get(i).getDistance(mouseLocation) <= convertPixelsToUnits(20)) {
                Waypoint lastWaypoint = this.purePursuit.getWaypoint(i);
                this.purePursuit.setWaypoint(i, new Waypoint(mouseLocation, lastWaypoint));
                break;
            }
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyChar() == 'r' || e.getKeyChar() == 'R') {
            this.purePursuit.reset();
            this.velocities.clear();
            this.drivingAngles.clear();
        } else if (e.getKeyChar() == 't' || e.getKeyChar() == 'T') {
            this.purePursuit.setRunning(!this.purePursuit.isRunning());
        }

    }

    @Override
    public void mousePressed(MouseEvent e) {
        System.out.println("Clicked (" + convertPixelsToX(e.getX(), DIMENSION) + "," + convertPixelsToY(e.getY(), DIMENSION) + ")");
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        System.out.println("Released (" + convertPixelsToX(e.getX(), DIMENSION) + "," + convertPixelsToY(e.getY(), DIMENSION) + ")");

        System.out.println(this.obstacleAvoiding.isThereAnObstacle(this.purePursuit.getStartWaypoint(), this.purePursuit.getFinalWaypoint()));
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
        return (DIMENSION.getX() / maxValue) / 2;
    }
}
