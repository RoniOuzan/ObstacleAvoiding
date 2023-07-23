package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.settings.Settings;

import javax.swing.*;
import java.awt.*;

public class Checklist extends JCheckBox implements InputComponent {
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

    @Override
    public void setLocation(int lastY) {
        this.setLocation(Settings.GAP, Settings.GAP + lastY);
    }
}
