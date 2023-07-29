package obstacleavoiding.path.robot;

import obstacleavoiding.math.geometry.Pose2d;

public record RobotState(Pose2d pose, double velocity, double driftPercent, double slowPercent, double curvature) {
}
