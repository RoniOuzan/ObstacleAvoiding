package obstacleavoiding.path;

import obstacleavoiding.gui.input.TextField;
import obstacleavoiding.gui.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.Map;

public class Settings extends JPanel {

    private static final int TEXT_HEIGHT = 20;
    private static final double TEXT_WIDTH_PERCENT = 0.45;
    private static final double TEXT_FIELD_WIDTH_PERCENT = 0.45;

    private final Map<String, TextField<?>> map = new HashMap<>();

    public Settings(int width, Map<String, Object> map) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), 0);
        this.setSize(width, GUI.FIELD_DIMENSION.getY());

        int gap = (int) (width * (1 - TEXT_WIDTH_PERCENT - TEXT_FIELD_WIDTH_PERCENT)) / 2;

        this.setBackground(Color.DARK_GRAY);
        int i = 0;
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            int y = gap + (i * gap) + (i * TEXT_HEIGHT);
            Text text = new Text(entry.getKey() + ":", new Dimension2d((int) (width * TEXT_FIELD_WIDTH_PERCENT), TEXT_HEIGHT), new Dimension2d(gap, y))
                    .setTextSize((int) (TEXT_HEIGHT * 0.5)).setTextColor(Color.WHITE);

            TextField<?> textField = new TextField<>(new Dimension2d((int) (width * TEXT_WIDTH_PERCENT), TEXT_HEIGHT), new Dimension2d(gap + (int) (width * TEXT_FIELD_WIDTH_PERCENT), y), entry.getValue());

            this.map.put(entry.getKey(), textField);
            this.add(textField);
            this.add(text);

            i++;
        }
    }

    public Object getValue(String name) {
        return this.map.get(name).getValue();
    }
}
