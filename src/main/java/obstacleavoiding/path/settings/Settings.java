package obstacleavoiding.path.settings;

import obstacleavoiding.gui.input.TextField;
import obstacleavoiding.gui.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.tables.TableType;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Settings extends JPanel {

    public static final int TEXT_HEIGHT = 20;
    public static final double TEXT_WIDTH_PERCENT = 0.45;
    public static final double TEXT_FIELD_WIDTH_PERCENT = 0.45;

    public static final Color BACKGROUND = Color.DARK_GRAY;

    private final Map<String, TableType<?>> map = new HashMap<>();

    public Settings(int width, List<TableType<?>> values) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), 0);
        this.setSize(width, GUI.FIELD_DIMENSION.getY());

        int gap = (int) (width * (1 - TEXT_WIDTH_PERCENT - TEXT_FIELD_WIDTH_PERCENT)) / 2;

        this.setBackground(BACKGROUND);
        for (int i = 0; i < values.size(); i++) {
            TableType<?> table = values.get(i);

            this.map.put(table.getName(), table);
            table.getComponents(i == 0 ? 0 : values.get(i - 1).getLastY(), gap).forEach(this::add);
        }
    }

    public void update() {
        this.map.values().forEach(t -> {
            t.update();
            t.parse();
        });
    }

    public Object getValue(String name) {
        return this.map.get(name).getValue();
    }

    public void setValue(String name, Object value) {
        this.map.get(name).setValue(value);
    }
}
