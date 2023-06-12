package obstacleavoiding.gui.components.output;

import obstacleavoiding.math.geometry.Dimension2d;

import java.awt.*;

public class Text extends Label implements OutputComponent {
    public Text(Dimension2d size, Dimension2d location, String text) {
        super(text);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setFont(new Font("ariel", Font.BOLD, 10));
    }

    public Text setTextSize(int size) {
        this.setFont(new Font("ariel", Font.BOLD, size));
        return this;
    }

    public Text setBackgroundColor(Color color) {
        this.setBackground(color);
        return this;
    }

    public Text setTextColor(Color color) {
        this.setForeground(color);
        return this;
    }
}
