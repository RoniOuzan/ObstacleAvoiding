package obstacleavoiding.gui.input;

import obstacleavoiding.gui.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import java.awt.*;

public class Checklist extends JCheckBox {
    public Checklist(Dimension2d size, Dimension2d location, String text, boolean defaultValue) {
        super(text, defaultValue);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setFocusable(false);
    }

    public Checklist setTextSize(int size) {
        this.setFont(new Font("ariel", Font.BOLD, size));
        return this;
    }

    public Checklist setBackgroundColor(Color color) {
        this.setBackground(color);
        return this;
    }

    public Checklist setTextColor(Color color) {
        this.setForeground(color);
        return this;
    }
}
