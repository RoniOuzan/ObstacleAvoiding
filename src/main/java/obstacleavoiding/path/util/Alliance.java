package obstacleavoiding.path.util;

import obstacleavoiding.path.obstacles.Obstacle;

import java.awt.*;

public enum Alliance {
    RED(new Color(255, 0, 0)),
    BLUE(new Color(0, 0, 255)),
    NONE(new Color(0, 0, 0));

    private final Color color;

    Alliance(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return color;
    }

    public Color getColor(int alpha) {
        return new Color(this.color.getRed(), this.color.getGreen(), this.color.getBlue(), alpha);
    }

    public Alliance getOther() {
        if (this == RED)
            return BLUE;
        else if (this == BLUE)
            return RED;
        return NONE;
    }
}
