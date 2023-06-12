package obstacleavoiding.path.settings;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.tables.TableType;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Settings extends JPanel {

    public static final double SETTINGS_HEIGHT_PERCENT = 0.6;
    public static final double GAP_PERCENT = 0.1;

    public static final Color BACKGROUND = Color.DARK_GRAY;

    private final Map<String, TableType<?>> map = new HashMap<>();

    public Settings(List<TableType<?>> values) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), 0);
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));

        int gap = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

        this.setBackground(BACKGROUND);
        for (int i = 0; i < values.size(); i++) {
            TableType<?> table = values.get(i);

            this.map.put(table.getName(), table);
            table.getComponents(i == 0 ? 0 : values.get(i - 1).getLastY(), gap).forEach(this::add);
        }
    }

    private void add(Component component) {
        super.add((java.awt.Component) component);
    }

    public void update() {
        this.map.values().forEach(t -> {
            t.update();
            t.parse();
        });
    }

    public Object getValue(String name, Object defaultValue) {
        if (this.map.containsKey(name))
            return this.map.get(name).getValue();
        return defaultValue;
    }

    public void setValue(String name, Object value) {
        this.map.get(name).setValue(value);
    }
}
