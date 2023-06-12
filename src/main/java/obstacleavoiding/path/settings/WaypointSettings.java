package obstacleavoiding.path.settings;

import static obstacleavoiding.path.settings.Settings.*;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.input.MultipleOption;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.tables.InputTable;
import obstacleavoiding.path.settings.tables.SelectOptionTable;
import obstacleavoiding.path.settings.tables.SliderTable;
import obstacleavoiding.path.settings.tables.TableType;
import obstacleavoiding.path.util.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.function.Supplier;

public class WaypointSettings extends JPanel {

    private final Supplier<Waypoint> waypoint;
    private final List<TableType<?>> tables;
    private final Map<String, TableType<?>> map = new HashMap<>();

    private Waypoint lastWaypoint = null;

    public WaypointSettings(Supplier<Waypoint> waypoint) {
        this.waypoint = waypoint;

        this.setLocation(GUI.FIELD_DIMENSION.getX(), (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * (1 - SETTINGS_HEIGHT_PERCENT)));

        this.tables = Arrays.asList(
                new SliderTable("Heading", 0, 0, 360)
        );

        this.setBackground(BACKGROUND);
        this.addAll();
    }

    @Override
    protected void paintComponent(Graphics g) {
        g.setColor(BACKGROUND);
        g.fillRect(0, 0, this.getWidth(), this.getHeight());

        g.setColor(BACKGROUND.brighter());
        g.fillRect(0, 0, this.getWidth(), 10);
    }

    public void addAll() {
        int gap = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

        for (int i = 0; i < this.tables.size(); i++) {
            TableType<?> table = this.tables.get(i);

            this.map.put(table.getName(), table);
            table.getComponents(i == 0 ? 0 : this.tables.get(i - 1).getLastY(), gap).forEach(this::add);
        }
    }

    private void add(Component component) {
        super.add((java.awt.Component) component);
    }

    public void update() {
        if (this.waypoint.get() != null && this.lastWaypoint == null) {
            this.addAll();

            this.setValue("Heading", this.waypoint.get().getHeading());
            this.getTable("Heading").setValueParser(h -> this.waypoint.get().setHeading((double) h));
        }

        if (this.waypoint.get() != null) {
            this.map.values().forEach(t -> {
                t.update();
                t.parse();
            });
        } else {
            this.removeAll();
        }

        this.lastWaypoint = waypoint.get();
    }

    public TableType<?> getTable(String name) {
        return this.map.get(name);
    }

    public void setValue(String name, Object value) {
        this.map.get(name).setValue(value);
    }
}
