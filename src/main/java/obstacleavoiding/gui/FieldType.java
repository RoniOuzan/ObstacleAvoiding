package main.java.obstacleavoiding.gui;

import main.java.obstacleavoiding.math.geometry.Dimension2d;

public interface FieldType {
    default int convertX(double x, Dimension2d dimension) {
        return (int) x;
    }

    default int convertY(double y, Dimension2d dimension) {
        return (int) y;
    }

    default double convertPixelsToX(double pixels, Dimension2d dimension) {
        return convertPixelsToUnits(pixels - 7);
    }

    default double convertPixelsToY(double pixels, Dimension2d dimension) {
        return convertPixelsToUnits(pixels + 30);
    }

    double convertPixelsToUnits(double pixels);
}
