package obstacleavoiding.path.settings;

import static obstacleavoiding.path.settings.Settings.*;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.MultipleOption;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.tables.SelectOptionTable;
import obstacleavoiding.path.settings.tables.SliderTable;
import obstacleavoiding.path.settings.tables.TableType;
import obstacleavoiding.path.util.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;

public class WaypointSettings extends JPanel {

    private final List<TableType<?>> tables;
    private final Map<String, TableType<?>> map = new HashMap<>();

    private Waypoint lastWaypoint = null;

    public WaypointSettings() {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * (1 - SETTINGS_HEIGHT_PERCENT)));
        this.setLayout(new GroupLayout(this));

        this.tables = Arrays.asList(
                new SliderTable("Heading", 0, -180, 180),
                new SelectOptionTable<>("RobotReference", Waypoint.RobotReference.CENTER, Waypoint.RobotReference.values())
        );

        this.setBackground(BACKGROUND);
    }

    @Override
    protected void paintComponent(Graphics g) {
        g.setColor(BACKGROUND);
        g.fillRect(0, 0, this.getWidth(), this.getHeight());

        g.setColor(GUI.COLOR);
        g.fillRect(0, 0, this.getWidth(), 8);
    }

    public void addAll() {
        int gap = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

        for (int i = 0; i < this.tables.size(); i++) {
            TableType<?> table = this.tables.get(i);

            this.map.put(table.getName(), table);
            table.getComponents(i == 0 ? 0 : this.tables.get(i - 1).getLastY(), gap).forEach(this::add);
        }
    }

    @Override
    public void removeAll() {
        super.removeAll();
        this.map.clear();
    }

    private void add(Component component) {
        super.add((java.awt.Component) component);
    }

    public void update(Waypoint waypoint) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setFocusable(false);

        if (waypoint != null && waypoint != this.lastWaypoint) {
            this.removeAll();
            this.addAll();

            this.setValue("Heading", waypoint.getHeading());
//            this.getTable("Heading").setValueParser(h -> waypoint.setHeading((double) h));

            this.setValue("RobotReference", waypoint.getRobotReference());
//            this.getTable("RobotReference").setValueParser(r -> waypoint.setRobotReference((Waypoint.RobotReference) r));
        }

        if (waypoint != null) {
            this.map.values().forEach(t -> {
                t.update();

                waypoint.setHeading((double) this.getTable("Heading").getValue());
                waypoint.setRobotReference((Waypoint.RobotReference) this.getTable("RobotReference").getValue());
            });
        } else {
            this.removeAll();
        }

        this.lastWaypoint = waypoint;
    }

    public TableType<?> getTable(String name) {
        return this.map.get(name);
    }

    public void setValue(String name, Object value) {
        this.map.get(name).setValue(value);
    }
}
