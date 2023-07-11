package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.TextField;
import obstacleavoiding.gui.components.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
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
    public T getCurrentValue() {
        return (T) textField.getValue();
    }

    @Override
    public void setCurrentValue(Object value) {
        this.textField.setValue(value);
    }

    @Override
    public List<Component> getComponents(int i, int gap) {
        int y = gap + (i * gap) + (i * TEXT_HEIGHT);
        Text text = new Text(new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_WIDTH_PERCENT), TEXT_HEIGHT), new Dimension2d(gap, y), this.getName() + ":")
                .setTextSize((int) (TEXT_HEIGHT * 0.5)).setTextColor(Color.WHITE);
        this.textField = new TextField<>(new Dimension2d((int) (GUI.SETTINGS_WIDTH * TEXT_FIELD_WIDTH_PERCENT), TEXT_HEIGHT), new Dimension2d(gap + (int) (GUI.SETTINGS_WIDTH * TEXT_FIELD_WIDTH_PERCENT), y), this.getDefaultValue());

        return Arrays.asList(this.textField, text);
    }
}
