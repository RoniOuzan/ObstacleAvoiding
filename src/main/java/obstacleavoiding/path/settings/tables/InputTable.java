package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.TextField;
import obstacleavoiding.gui.components.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.awt.*;
import java.util.Arrays;
import java.util.List;

public class InputTable<T> extends TableType<T> {
    private static final int TEXT_HEIGHT = 20;
    private static final double TEXT_WIDTH_PERCENT = 0.45;
    private static final double TEXT_FIELD_WIDTH_PERCENT = 0.45;

    private TextField<?> textField;

    public InputTable(String name, ValuesMode mode, T defaultValue) {
        super(name, mode, defaultValue);
    }

    @Override
    public int getLastY() {
        return this.textField.getY() + this.textField.getHeight();
    }

    @Override
    public T getValue() {
        return (T) textField.getValue();
    }

    @Override
    public void setValue(Object value) {
        this.textField.setValue(value);
    }

    @Override
    public List<Component> getComponents() {
        Text text = new Text(new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_WIDTH_PERCENT), TEXT_HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getName() + ":")
                .setTextSize((int) (TEXT_HEIGHT * 0.5)).setTextColor(Color.WHITE);
        this.textField = new TextField<>(new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_FIELD_WIDTH_PERCENT), TEXT_HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getDefaultValue());

        return Arrays.asList(this.textField, text);
    }
}
