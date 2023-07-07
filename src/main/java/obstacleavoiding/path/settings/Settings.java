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

    public static final double SETTINGS_HEIGHT_PERCENT = 0.9;
    public static final double GAP_PERCENT = 0.05;

    public static final Color BACKGROUND = Color.DARK_GRAY;

    private final Map<String, TableType<?>> map = new HashMap<>();

    public Settings(List<TableType<?>> values) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), 0);
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setBackground(BACKGROUND);
        this.setLayout(new GroupLayout(this));
        this.setFocusable(false);

        int gap = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

        this.setBackground(BACKGROUND);
        TableType<?> lastTable = null;
        for (TableType<?> table : values) {
            this.map.put(table.getName(), table);
            if (table.isChangeable()) {
                table.getComponents(lastTable == null ? 0 : lastTable.getLastY(), gap).forEach(this::add);
                lastTable = table;
            }
        }
    }

    private void add(Component component) {
        super.add((java.awt.Component) component);
    }

    public void update() {
        this.map.values().parallelStream().filter(TableType::isChangeable).forEach(t -> {
            t.update();
            t.parse();
        });
    }

    public <T> T getValue(String name, T defaultValue) {
        TableType<T> table = (TableType<T>) this.map.get(name);
        if (table != null)
            return table.getValue();
        return defaultValue;
    }

    public void setValue(String name, Object value) {
        this.map.get(name).setValue(value);
    }
}
