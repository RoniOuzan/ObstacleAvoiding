package obstacleavoiding.path.util;

import obstacleavoiding.path.obstacles.Obstacle;

import java.awt.*;

public enum Alliance {
    RED("Red", new Color(255, 0, 0)),
    BLUE("Blue", new Color(0, 0, 255)),
    NONE("None", new Color(0, 0, 0));

    private final String text;
    private final Color color;

    Alliance(String text, Color color) {
        this.text = text;
        this.color = color;
    }

    public String getText() {
        return text;
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
