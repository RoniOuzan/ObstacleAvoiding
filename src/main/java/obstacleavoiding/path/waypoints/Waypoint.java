package obstacleavoiding.path.waypoints;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.GUI;

public class Waypoint {
    private double x;
    private double y;

    private Rotation2d heading;

    private RobotReferencePoint robotReference;

    public Waypoint(double x, double y, Rotation2d heading, RobotReferencePoint robotReference) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.robotReference = robotReference;
    }

    public Waypoint(double x, double y, RobotReferencePoint robotReference) {
        this(x, y, new Rotation2d(), robotReference);
    }

    public Waypoint(double distance, Rotation2d angle, Rotation2d heading, RobotReferencePoint robotReference) {
        this(new Translation2d(distance, angle), heading, robotReference);
    }

    public Waypoint(Translation2d translation2d, Rotation2d heading, RobotReferencePoint robotReference) {
        this(translation2d.getX(), translation2d.getY(), heading, robotReference);
    }

    public Waypoint(Translation2d translation2d, RobotReferencePoint robotReference) {
        this(translation2d.getX(), translation2d.getY(), new Rotation2d(), robotReference);
    }

    public Waypoint(Pose2d pose2d, RobotReferencePoint robotReference) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getRotation(), robotReference);
    }

    public Waypoint(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getRotation(), RobotReferencePoint.CENTER);
    }

    public void set(Pose2d pose2d) {
        this.set(pose2d.getTranslation());
        this.setHeading(pose2d.getRotation());
    }

    public void set(Translation2d translation2d) {
        this.x = translation2d.getX();
        this.y = translation2d.getY();
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }

    public RobotReferencePoint getRobotReference() {
        return robotReference;
    }

    public void setRobotReference(RobotReferencePoint robotReference) {
        this.robotReference = robotReference;
    }

    public Translation2d getRobotReferenceTranslation() {
        return RobotReferencePoint.CENTER
                .getTranslation2d()
                .minus(robotReference.getTranslation2d())
                .rotateBy(heading);
    }

    public Translation2d getReferencedPosition() {
        return this.getTranslation2d().plus(this.getRobotReferenceTranslation());
    }

    public double getDistance(Waypoint other) {
        return Math.hypot(this.x - other.x, this.y - other.y);
    }

    public Translation2d getTranslation2d() {
        return new Translation2d(this.x, this.y);
    }

    public Pose2d getPose2d() {
        return new Pose2d(this.getTranslation2d(), this.heading);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x - getRobotReferenceTranslation().getX();
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y - getRobotReferenceTranslation().getY();
    }

    @Override
    public String toString() {
        return "(" + (Math.round(this.getX() * 100) / 100d) + "," + (Math.round(this.getY() * 100) / 100d) + "," + (Math.round(this.getHeading().getDegrees() * 100) / 100d) + ")";
    }

    @Override
    public boolean equals(Object obj) {
        return this == obj;
    }

    public enum RobotReferencePoint {
        FRONT_LEFT(GUI.ROBOT_WIDTH, GUI.ROBOT_WIDTH),
        FRONT_CENTER(GUI.ROBOT_WIDTH, GUI.HALF_ROBOT),
        FRONT_RIGHT(GUI.ROBOT_WIDTH, 0),
        BACK_LEFT(0, GUI.ROBOT_WIDTH),
        BACK_CENTER(0, GUI.HALF_ROBOT),
        BACK_RIGHT(0, 0),
        CENTER(GUI.HALF_ROBOT, GUI.HALF_ROBOT)
        ;

        private final Translation2d translation2d;

        RobotReferencePoint(double x, double y) {
            this.translation2d = new Translation2d(x, y);
        }

        public Translation2d getTranslation2d() {
            return this.translation2d;
        }
    }
}
