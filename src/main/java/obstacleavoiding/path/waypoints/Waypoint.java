package obstacleavoiding.path.waypoints;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.GUI;

public class Waypoint extends Translation2d {
    private double heading;

    private RobotReference robotReference;

    public Waypoint(double x, double y, double heading, RobotReference robotReference) {
        super(x, y);
        this.heading = heading;
        this.robotReference = robotReference;
    }

    public Waypoint(double x, double y, RobotReference robotReference) {
        this(x, y, 0, robotReference);
    }

    public Waypoint(double distance, Rotation2d angle, double heading, RobotReference robotReference) {
        this(new Translation2d(distance, angle), heading, robotReference);
    }

    public Waypoint(Translation2d translation2d, double heading, RobotReference robotReference) {
        this(translation2d.getX(), translation2d.getY(), heading, robotReference);
    }

    public Waypoint(Translation2d translation2d, RobotReference robotReference) {
        this(translation2d.getX(), translation2d.getY(), 0, robotReference);
    }

    public void set(Pose2d pose2d) {
        super.set(pose2d.getTranslation());
        this.setHeading(pose2d.getRotation().getDegrees());
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public Pose2d getPose2d() {
        return new Pose2d(this, Rotation2d.fromDegrees(this.heading));
    }

    public RobotReference getRobotReference() {
        return robotReference;
    }

    public void setRobotReference(RobotReference robotReference) {
        this.robotReference = robotReference;
    }

    public Translation2d getRobotReferenceTranslation() {
        return robotReference.getReference(Rotation2d.fromDegrees(this.heading));
    }

    public Translation2d getReferencedPosition() {
        return this.minus(this.getRobotReferenceTranslation());
    }

    @Override
    public void setX(double x) {
        super.setX(x - robotReference.getReference(Rotation2d.fromDegrees(heading)).getX());
    }

    @Override
    public void setY(double y) {
        super.setY(y - robotReference.getReference(Rotation2d.fromDegrees(heading)).getY());
    }

    @Override
    public String toString() {
        return "(" + this.getX() + "," + this.getY() + "," + this.getHeading() + ")";
    }

    @Override
    public boolean equals(Object obj) {
        return this == obj;
    }

    public enum RobotReference {
        FRONT_LEFT(GUI.HALF_ROBOT, GUI.HALF_ROBOT),
        FRONT_CENTER(GUI.HALF_ROBOT, 0),
        FRONT_RIGHT(GUI.HALF_ROBOT, -GUI.HALF_ROBOT),
        BACK_LEFT(-GUI.HALF_ROBOT, GUI.HALF_ROBOT),
        BACK_CENTER(-GUI.HALF_ROBOT, 0),
        BACK_RIGHT(-GUI.HALF_ROBOT, -GUI.HALF_ROBOT),
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
