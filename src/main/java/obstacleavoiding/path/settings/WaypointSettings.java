package obstacleavoiding.path.settings;

import static obstacleavoiding.path.settings.Settings.*;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.robot.Robot;
import obstacleavoiding.path.settings.tables.SelectOptionTable;
import obstacleavoiding.path.settings.tables.DoubleSliderTable;
import obstacleavoiding.path.settings.tables.TableType;
import obstacleavoiding.path.util.ValuesMode;
import obstacleavoiding.path.waypoints.NavigationWaypoint;
import obstacleavoiding.path.waypoints.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;

public class WaypointSettings extends JPanel {

    private final List<TableType<?>> tables;
    private final List<TableType<?>> navigationTables;
    private final Map<String, TableType<?>> map = new HashMap<>();

    private Waypoint lastWaypoint = null;

    public WaypointSettings(Robot robot) {
        this.setLocation(GUI.FIELD_DIMENSION.getX(), (int) (GUI.FIELD_DIMENSION.getY() * SETTINGS_HEIGHT_PERCENT));
        this.setSize(GUI.SETTINGS_WIDTH, (int) (GUI.FIELD_DIMENSION.getY() * (1 - SETTINGS_HEIGHT_PERCENT)));
        this.setLayout(new GroupLayout(this));

        this.tables = Arrays.asList(
                new DoubleSliderTable("Heading", ValuesMode.ALL, 0, -180, 180),
                new SelectOptionTable<>("RobotReference", ValuesMode.ALL, Waypoint.RobotReferencePoint.CENTER, Waypoint.RobotReferencePoint.values())
        );
        this.navigationTables = Arrays.asList(
                new DoubleSliderTable("TargetVel", ValuesMode.ALL, 1, 0, robot.getConstants().maxVel())
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

    public void addAll(boolean navigation) {
        int gap = (int) (GUI.SETTINGS_WIDTH * GAP_PERCENT) / 2;

        for (int i = 0; i < this.tables.size(); i++) {
            TableType<?> table = this.tables.get(i);

            this.map.put(table.getName(), table);
            int finalI = i;
            table.getComponents().forEach(c -> {
                c.setLocation(finalI == 0 ? 0 : this.tables.get(finalI - 1).getLastY());
                ((java.awt.Component) c).setName(table.getName());
                this.add(c);
            });
        }

        if (navigation) {
            for (int i = 0; i < this.navigationTables.size(); i++) {
                TableType<?> table = this.navigationTables.get(i);

                this.map.put(table.getName(), table);
                int finalI = i;
                table.getComponents().forEach(c -> {
                    c.setLocation(finalI == 0 ? this.tables.get(this.tables.size() - 1).getLastY() : this.navigationTables.get(finalI - 1).getLastY());
                    ((java.awt.Component) c).setName(table.getName());
                    this.add(c);
                });
            }
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
            this.addAll(waypoint instanceof NavigationWaypoint);

            this.setValue("Heading", waypoint.getHeading().getDegrees());
            this.setValue("RobotReference", waypoint.getRobotReference());
            if (waypoint instanceof NavigationWaypoint navigation) {
                this.setValue("TargetVel", navigation.getTargetVelocity());
            }
        }

        if (waypoint != null) {
            this.map.values().forEach(t -> {
                t.update();

                waypoint.setHeading(Rotation2d.fromDegrees((double) this.getTable("Heading").getValue()));
                waypoint.setRobotReference((Waypoint.RobotReferencePoint) this.getTable("RobotReference").getValue());

                if (waypoint instanceof NavigationWaypoint navigation) {
                    navigation.setTargetVelocity((double) this.getTable("TargetVel").getValue());
                }
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
