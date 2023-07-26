/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package obstacleavoiding.math.util;

import obstacleavoiding.util.LimitedSizeQueue;

/**
 * This class is an implementation of a smoothing noise out of data.
 */
public class KalmanFilter {

    private int windowSize;
    private LimitedSizeQueue<Double> queue;
    private double sumValues = 0;

    /**
     * @param windowSize The size of the flitter.
     */
    public KalmanFilter(int windowSize) {
        this.windowSize = windowSize;
        this.queue = new LimitedSizeQueue<Double>(windowSize);
    }

    /**
     * @param newValue the new data from the sensor.
     */
    public void addValue(double newValue) {
        sumValues += newValue;
        if (queue.size() == windowSize) {
            sumValues -= queue.getOldest();
        }
        queue.add(newValue);
    }

    /**
     * @return the smooth value.
     */
    public double getSmoothValue() {
        return (queue.size() > 0) ? sumValues / queue.size() : 0;
    }

    public void reset() {
        queue.clear();
        sumValues = 0;
    }
}