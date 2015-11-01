package com.example.wireless.indoorlocalization;

/**
 * Created by wireless on 2015-10-12.
 */
public class Location2D {
    private float x, y;

    public Location2D(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Location2D(double x, double y) {
        this.x = (float) x;
        this.y = (float) y;
    }

    public void setLocation(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void setLocation(Location2D loc) {
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
