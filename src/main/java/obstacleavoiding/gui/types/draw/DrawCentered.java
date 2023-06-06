package obstacleavoiding.gui.types.draw;

import obstacleavoiding.gui.DrawType;

public interface DrawCentered extends DrawType {
    @Override
    default int convertXWithSize(double x, double size) {
        return (int) (x - (size / 2));
    }

    @Override
    default int convertYWithSize(double y, double size) {
        return (int) (y - (size / 2));
    }

    @Override
    default int convertHeight(double height) {
        return (int) -height;
    }
}
