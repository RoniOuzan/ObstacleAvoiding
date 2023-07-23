package obstacleavoiding.path.settings;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.tables.SelectOptionTable;
import obstacleavoiding.path.settings.tables.TableType;
import obstacleavoiding.path.util.ValuesMode;
import obstacleavoiding.util.Entry;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Settings extends JPanel {

    public static final double SETTINGS_HEIGHT_PERCENT = 0.89;
    public static final double GAP_PERCENT = 0.05;
    public static final int GAP = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

    public static final Color BACKGROUND = Color.DARK_GRAY;

    private final List<TableType<?>> values;
    private final Map<String, Entry<TableType<?>, List<Component>>> map = new HashMap<>();

    private ValuesMode mode = ValuesMode.ALL;

    public Settings(List<TableType<?>> values) {
        this.values = values;

        this.setLocation(GUI.FIELD_DIMENSION.getX(), 0);
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setBackground(BACKGROUND);
        this.setLayout(new GroupLayout(this));
        this.setFocusable(false);

        this.setBackground(BACKGROUND);
        values.add(0, new SelectOptionTable<>("Mode", ValuesMode.ALL, mode, ValuesMode.values())
                .onChange((c, l) -> {
                    for (TableType<?> table : values) {
                        table.setChangeable(table.getMode() == ValuesMode.ALL || c == ValuesMode.ALL || table.getMode() == c);
                    }
                }));
        TableType<?> lastTable = null;
        for (TableType<?> table : values) {
            List<Component> components = table.getComponents();
            this.map.put(table.getName(), new Entry<>(table, components));
            if (table.isChangeable()) {
                TableType<?> finalLastTable = lastTable;

                components.forEach(c -> {
                    c.setLocation(finalLastTable == null ? 0 : finalLastTable.getLastY());
                    ((java.awt.Component) c).setName(table.getName());
                    this.add(c);
                });

                lastTable = table;
            }

        }
    }

    private void refresh() {
        this.removeAll();

        TableType<?> lastTable = null;
        for (TableType<?> table : values) {
            if (table.isChangeable()) {
                TableType<?> finalLastTable = lastTable;
                this.map.get(table.getName()).b().forEach(c -> {
                    c.setLocation(finalLastTable == null ? 0 : finalLastTable.getLastY());
                    ((java.awt.Component) c).setName(table.getName());
                    this.add(c);
                });

                lastTable = table;
            }
        }
    }

    private void add(Component component) {
        super.add((java.awt.Component) component);
    }

    public void update() {
        this.map.values().parallelStream().map(Entry::a).filter(t -> t.isChangeable() || t.getName().equals("Mode")).forEach(t -> {
            t.update();
            t.parse();
        });

        ValuesMode mode = this.getValue("Mode", ValuesMode.ALL);
        if (this.mode != mode) {
            this.refresh();
            this.setValue("Mode", mode);
            this.mode = mode;
        }
    }

    public <T> T getValue(String name, T defaultValue) {
        TableType<T> table = (TableType<T>) this.map.get(name).a();
        if (table != null)
            return table.getValue();
        return defaultValue;
    }

    public void setValue(String name, Object value) {
        this.map.get(name).a().setValue(value);
    }
}
