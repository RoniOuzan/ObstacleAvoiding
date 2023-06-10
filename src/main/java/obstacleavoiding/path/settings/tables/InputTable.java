package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.input.TextField;
import obstacleavoiding.gui.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;

import java.awt.*;
import java.util.Arrays;
import java.util.List;

public class InputTable<T> extends TableType<T> {
    public static final double TEXT_WIDTH_PERCENT = 0.45;
    public static final double TEXT_FIELD_WIDTH_PERCENT = 0.45;

    private TextField<?> textField;

    public InputTable(String name, T defaultValue) {
        super(name, defaultValue);
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
    public List<Component> getComponents(int i, int gap) {
        int y = gap + (i * gap) + (i * Settings.TEXT_HEIGHT);
        Text text = new Text(this.getName() + ":", new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_WIDTH_PERCENT), Settings.TEXT_HEIGHT), new Dimension2d(gap, y))
                .setTextSize((int) (Settings.TEXT_HEIGHT * 0.5)).setTextColor(Color.WHITE);
        this.textField = new TextField<>(new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_FIELD_WIDTH_PERCENT), Settings.TEXT_HEIGHT), new Dimension2d(gap + (int) (GUI.SETTINGS_WIDTH * TEXT_FIELD_WIDTH_PERCENT), y), this.getDefaultValue());

        return Arrays.asList(this.textField, text);
    }
}
