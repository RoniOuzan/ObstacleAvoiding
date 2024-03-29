package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.Checklist;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.awt.*;
import java.util.Collections;
import java.util.List;

public class BooleanTable extends TableType<Boolean> {

    private static final int HEIGHT = 20;

    private Checklist checklist;

    private boolean lastState;

    private Runnable onTrue = () -> {};
    private Runnable onFalse = () -> {};
    private Runnable whileTrue = () -> {};
    private Runnable whileFalse = () -> {};
    private Runnable onToggle = () -> {};

    private Boolean alwaysState = null;

    public BooleanTable(String name, ValuesMode mode, boolean defaultValue) {
        super(name, mode, defaultValue);
        this.lastState = defaultValue;
    }

    public BooleanTable onTrue(Runnable runnable) {
        this.onTrue = runnable;
        return this;
    }

    public BooleanTable onFalse(Runnable runnable) {
        this.onFalse = runnable;
        return this;
    }

    public BooleanTable whileTrue(Runnable runnable) {
        this.whileTrue = runnable;
        return this;
    }

    public BooleanTable whileFalse(Runnable runnable) {
        this.whileFalse = runnable;
        return this;
    }

    public BooleanTable onToggle(Runnable runnable) {
        this.onToggle = runnable;
        return this;
    }

    public BooleanTable setAlways(Boolean always) {
        this.alwaysState = always;
        return this;
    }

    @Override
    public int getLastY() {
        return this.checklist.getY() + this.checklist.getHeight();
    }

    @Override
    public Boolean getValue() {
        return this.checklist.isSelected();
    }

    @Override
    public void setValue(Object value) {
        this.checklist.setSelected((Boolean) value);
    }

    @Override
    public List<Component> getComponents() {
        this.checklist = new Checklist(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * Settings.GAP), HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getName(),
                this.getDefaultValue()
        ).setTextSize((int) (HEIGHT * 0.8)).setTextColor(Color.WHITE).setBackgroundColor(Settings.BACKGROUND);

        return Collections.singletonList(this.checklist);
    }

    @Override
    public void update() {
        boolean currentState = this.getValue();

        if (currentState ^ this.lastState) {
            this.onToggle.run();
        }

        if (currentState) {
            if (!this.lastState) {
                this.onTrue.run();
            } else {
                this.whileTrue.run();
            }
        } else {
            if (this.lastState) {
                this.onFalse.run();
            } else {
                this.whileFalse.run();
            }
        }

        if (this.alwaysState != null) {
            this.setValue(this.alwaysState);
            currentState = this.alwaysState;
        }

        this.lastState = currentState;
    }
}
