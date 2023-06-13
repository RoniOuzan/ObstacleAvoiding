package obstacleavoiding.gui.components.input;

import obstacleavoiding.gui.FrameUtil;
import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import javax.swing.plaf.basic.BasicSliderUI;
import java.awt.*;
import java.awt.geom.RoundRectangle2D;

public abstract class Slider extends JSlider implements InputComponent {
    private Color color = new Color(246, 146, 36);

    public Slider(Dimension2d size, Dimension2d location, String name, int value, int minimum, int maximum) {
        super(JSlider.HORIZONTAL, minimum, maximum, value);
        this.setName(name);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setPaintTrack(true);
        this.setFocusable(false);
        this.setFont(new Font("arial", Font.BOLD, 15));
    }

    public abstract double getCurrentValue();

    public Slider setBackgroundColor(Color color) {
        this.setBackground(color);
        return this;
    }

    public Slider setColor(Color color) {
        this.color = color;
        return this;
    }

    @Override
    public void updateUI() {
        setUI(new CustomSliderUI());
    }

    private class CustomSliderUI extends BasicSliderUI {

        private static final int TRACK_HEIGHT = 8;
        private static final int TRACK_WIDTH = 8;
        private static final int TRACK_ARC = 5;
        private static final Dimension THUMB_SIZE = new Dimension(20, 20);
        private static final Font FONT = new Font("Ariel", Font.BOLD, 15);
        private final RoundRectangle2D.Float trackShape = new RoundRectangle2D.Float();

        public CustomSliderUI() {
            super(Slider.this);
        }

        @Override
        protected void calculateTrackRect() {
            super.calculateTrackRect();
            if (isHorizontal()) {
                trackRect.y = trackRect.y + (trackRect.height - TRACK_HEIGHT) / 2;
                trackRect.height = TRACK_HEIGHT;
            } else {
                trackRect.x = trackRect.x + (trackRect.width - TRACK_WIDTH) / 2;
                trackRect.width = TRACK_WIDTH;
            }
            trackShape.setRoundRect(trackRect.x, trackRect.y, trackRect.width, trackRect.height, TRACK_ARC, TRACK_ARC);
        }

        @Override
        protected void calculateThumbLocation() {
            super.calculateThumbLocation();
            if (isHorizontal()) {
                thumbRect.y = trackRect.y + (trackRect.height - thumbRect.height) / 2;
            } else {
                thumbRect.x = trackRect.x + (trackRect.width - thumbRect.width) / 2;
            }
        }

        @Override
        protected Dimension getThumbSize() {
            return THUMB_SIZE;
        }

        private boolean isHorizontal() {
            return slider.getOrientation() == JSlider.HORIZONTAL;
        }

        @Override
        public void paint(final Graphics g, final JComponent c) {
            ((Graphics2D) g).setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            super.paint(g, c);
        }

        @Override
        public void paintTrack(final Graphics g) {
            Graphics2D g2 = (Graphics2D) g;
            Shape clip = g2.getClip();

            g2.setColor(Color.WHITE);
            g2.setFont(FONT);
            String nameText = Slider.this.getName() + ":";
            g2.drawString(nameText, 8, g2.getFont().getSize());

            double current = Slider.this.getCurrentValue();
            String value = current % 1 == 0 ? Integer.toString((int) current) : Double.toString(current);
            g2.drawString(value, 8 + FrameUtil.getStringLength(nameText + "  ", g2.getFont()), g2.getFont().getSize());

            boolean horizontal = isHorizontal();
            boolean inverted = slider.getInverted();

            // Paint shadow.
            g2.setColor(new Color(170, 170 ,170));
            g2.fill(trackShape);

            // Paint track background.
            g2.setColor(new Color(200, 200 ,200));
            g2.setClip(trackShape);
            trackShape.y += 1;
            g2.fill(trackShape);
            trackShape.y = trackRect.y;

            g2.setClip(clip);

            // Paint selected track.
            if (horizontal) {
                boolean ltr = slider.getComponentOrientation().isLeftToRight();
                if (ltr) inverted = !inverted;
                int thumbPos = thumbRect.x + thumbRect.width / 2;
                if (inverted) {
                    g2.clipRect(0, 0, thumbPos, slider.getHeight());
                } else {
                    g2.clipRect(thumbPos, 0, slider.getWidth() - thumbPos, slider.getHeight());
                }

            } else {
                int thumbPos = thumbRect.y + thumbRect.height / 2;
                if (inverted) {
                    g2.clipRect(0, 0, slider.getHeight(), thumbPos);
                } else {
                    g2.clipRect(0, thumbPos, slider.getWidth(), slider.getHeight() - thumbPos);
                }
            }
            g2.setColor(Color.ORANGE);
            g2.fill(trackShape);
            g2.setClip(clip);

            g2.setColor(Color.WHITE);

            String minimum = (Slider.this.getMinimum() / 10d) % 1 == 0 ? Integer.toString((int) (Slider.this.getMinimum() / 10d)) : Double.toString(Slider.this.getMinimum() / 10d);
            FrameUtil.drawCenteredString(g2, minimum, 12, slider.getHeight() - 3, FONT);

            String maximum = (Slider.this.getMaximum() / 10d) % 1 == 0 ? Integer.toString((int) (Slider.this.getMaximum() / 10d)) : Double.toString(Slider.this.getMaximum() / 10d);
            FrameUtil.drawCenteredString(g2, maximum, slider.getWidth() - 12, slider.getHeight() - 3, FONT);
        }

        @Override
        public void paintThumb(final Graphics g) {
            g.setColor(Slider.this.color);
            g.fillOval(thumbRect.x, thumbRect.y, thumbRect.width, thumbRect.height);
        }

        @Override
        public void paintFocus(final Graphics g) {
            g.setColor(Slider.this.color);
            g.fillOval(thumbRect.x, thumbRect.y, thumbRect.width, thumbRect.height);
        }
    }
}
