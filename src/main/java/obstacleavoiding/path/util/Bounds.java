package obstacleavoiding.path.util;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Translation2d;

public class Bounds {
    private final double maxX;
    private final double minX;
    private final double maxY;
    private final double minY;

    public Bounds(double maxX, double minX, double maxY, double minY) {
        this.maxX = maxX;
        this.minX = minX;
        this.maxY = maxY;
        this.minY = minY;
    }

    public Bounds(double x, double y) {
        this(x, -x, y, -y);
    }

    public boolean isInOfBounds(Translation2d translation2d) {
        return MathUtil.inRange(translation2d.getX(), this.maxX, this.minX) &&
                MathUtil.inRange(translation2d.getY(), this.maxY, this.minY);
    }
}
