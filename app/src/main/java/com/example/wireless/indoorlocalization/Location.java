package com.example.wireless.indoorlocalization;

import java.util.ArrayList;

/**
 * Created by wireless on 2015-10-12.
 */
public class Location {
    protected float x, y;

    public Location(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Location(double x, double y) {
        this.x = (float) x;
        this.y = (float) y;
    }

    public void setLocation(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void setLocation(Location loc) {
        this.x = loc.getX();
        this.y = loc.getY();
    }

    public void resetLocation() {
        this.x = 0;
        this.y = 0;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

}
