package obstacleavoiding.gui;

import java.awt.*;
import java.awt.font.FontRenderContext;
import java.awt.geom.Rectangle2D;

public class FrameUtil {
    public static void drawCenteredString(Graphics g, String s, int x, int y, Font font) {
        Rectangle2D bounds = font.getStringBounds(s, new FontRenderContext(null, true, true));

        g.setFont(font);
        g.drawString(s, (int) (x - (bounds.getWidth() / 2)), y);
    }
}
