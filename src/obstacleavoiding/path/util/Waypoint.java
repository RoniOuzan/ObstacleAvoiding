package obstacleavoiding.path.util;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.GUI;

public class Waypoint extends Translation2d {
    private final double heading;
    private final double movementAngle;

    private final RobotReference robotReference;

    public Waypoint(double x, double y, double heading, double movementAngle, RobotReference robotReference) {
        Translation2d translation2d = new Translation2d(x, y).plus(robotReference.getReference(Rotation2d.fromDegrees(heading)));
        this.m_x = translation2d.getX();
        this.m_y = translation2d.getY();
        this.heading = heading;
        this.movementAngle = movementAngle;
        this.robotReference = robotReference;
    }

    public Waypoint(double x, double y, RobotReference robotReference) {
        this(x, y, 0, 0, robotReference);
    }

    public Waypoint(double distance, Rotation2d angle, double heading, double movementAngle, RobotReference robotReference) {
        this(new Translation2d(distance, angle), heading, movementAngle, robotReference);
    }

    public Waypoint(Translation2d translation2d, double heading, double movementAngle, RobotReference robotReference) {
        this(translation2d.getX(), translation2d.getY(), heading, movementAngle, robotReference);
    }

    public Waypoint(Translation2d translation2d, Waypoint waypoint) {
        this(translation2d.getX(), translation2d.getY(), waypoint.heading, waypoint.movementAngle, waypoint.robotReference);
    }

    public Waypoint(Translation2d translation2d, double heading, RobotReference robotReference) {
        this(translation2d.getX(), translation2d.getY(), heading, 0, robotReference);
    }

    public Waypoint(Translation2d translation2d, RobotReference robotReference) {
        this(translation2d.getX(), translation2d.getY(), 0, 0, robotReference);
    }

    public double getHeading() {
        return heading;
    }

    public double getMovementAngle() {
        return movementAngle;
    }

    public RobotReference getRobotReference() {
        return robotReference;
    }

    public Translation2d getRobotReferenceTranslation() {
        return robotReference.getReference(Rotation2d.fromDegrees(heading));
    }

    public Translation2d getNormalTranslation(Rotation2d angle) {
        return this.minus(this.robotReference.getReference(angle));
    }


    @Override
    public String toString() {
        return "(" + this.getX() + "," + this.getY() + "," + this.getHeading() + "," + this.getMovementAngle() + ")";
    }

    public enum RobotReference {
        FRONT_LEFT(GUI.ROBOT_WITH_BUMPER, GUI.HALF_ROBOT),
        FRONT_CENTER(GUI.ROBOT_WITH_BUMPER, 0),
        FRONT_RIGHT(GUI.ROBOT_WITH_BUMPER, -GUI.HALF_ROBOT),
        BACK_LEFT(-GUI.ROBOT_WITH_BUMPER, GUI.HALF_ROBOT),
        BACK_CENTER(-GUI.ROBOT_WITH_BUMPER, 0),
        BACK_RIGHT(-GUI.ROBOT_WITH_BUMPER, -GUI.HALF_ROBOT),
        CENTER(0, 0)
        ;

        private final Translation2d translation2d;

        RobotReference(double x, double y) {
            this.translation2d = new Translation2d(x, y);
        }

        public Translation2d getReference(Rotation2d angle) {
            return this.translation2d.rotateBy(angle);
        }
    }
}
