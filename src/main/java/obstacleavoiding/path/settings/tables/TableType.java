package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;

import java.awt.*;
import java.util.List;
import java.util.function.Consumer;

public abstract class TableType<T> {
    private final String name;
    private final T defaultValue;

    private Consumer<T> valueParser = t -> {};

    public TableType(String name, T defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
    }

    public TableType<T> setValueParser(Consumer<T> valueParser) {
        this.valueParser = valueParser;
        return this;
    }

    public void parse() {
        this.valueParser.accept(this.getValue());
    }

    public String getName() {
        return name;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public abstract int getLastY();

    public abstract T getValue();
    public abstract void setValue(Object value);

    public abstract List<Component> getComponents(int lastY, int gap);

    public void update() {}
}
