package obstacleavoiding.gui;

import java.awt.*;
import java.awt.font.FontRenderContext;

public class FrameUtil {
    public static void drawCenteredString(Graphics g, String s, int x, int y, Font font) {
        g.setFont(font);
        g.drawString(s, (int) (x - (getStringLength(s, font) / 2d)), y);
    }

    public static int getStringLength(String text, Font font) {
        return (int) font.getStringBounds(text, new FontRenderContext(null, true, true)).getWidth();
    }
}
