package main.java.obstacleavoiding.gui.types.field;

import main.java.obstacleavoiding.math.geometry.Dimension2d;

public interface ZeroCenter extends ZeroLeftBottom {
    @Override
    default int convertX(double x, Dimension2d dimension) {
        return (int) (x + (dimension.getX() / 2d));
    }

    @Override
    default int convertY(double y, Dimension2d dimension) {
        return ZeroLeftBottom.super.convertY(y + (dimension.getY() / 2d), dimension);
    }

    @Override
    default double convertPixelsToX(double pixels, Dimension2d dimension) {
        return ZeroLeftBottom.super.convertPixelsToX(pixels - (dimension.getX() / 2d), dimension);
    }

    @Override
    default double convertPixelsToY(double pixels, Dimension2d dimension) {
        return ZeroLeftBottom.super.convertPixelsToYFieldType((dimension.getY() - pixels) - (dimension.getY() / 2d), dimension);
    }
}
