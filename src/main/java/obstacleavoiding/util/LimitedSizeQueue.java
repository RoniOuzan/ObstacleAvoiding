/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package obstacleavoiding.util;

import java.util.ArrayList;

public class LimitedSizeQueue<K> extends ArrayList<K> {

    private int maxSize;

    public LimitedSizeQueue(int size) {
        this.maxSize = size;
    }

    /**
     * Add new value the queue and remove the oldest
     */
    public boolean add(K k) {
        boolean r = super.add(k);
        if (size() > maxSize) {
            remove(0);
        }
        return r;
    }

    /**
     * @return the youngest value
     */
    public K getYoungest() {
        return get(size() - 1);
    }

    /**
     * @return the oldest value
     */
    public K getOldest() {
        return get(0);
    }
}
