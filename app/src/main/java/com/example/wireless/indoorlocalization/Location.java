package com.example.wireless.indoorlocalization;

import java.util.ArrayList;

/**
 * Created by wireless on 2015-10-12.
 */
public class Location {
    private final int WINDOWSIZE = 10;
    private float x, y;
    private ArrayList<Integer> rss_table = new ArrayList<>();
    private int avg_rss = 0;
    private int raw_rss = 0;

    public Location(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Location(double x, double y) {
        this.x = (float) x;
        this.y = (float) y;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
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

    public void addRSS(int r) {
        raw_rss = r;
        rss_table.add(raw_rss);
        if (rss_table.size() > WINDOWSIZE) {
            rss_table.remove(0);
        }
    }

    public boolean isCalibrated() {
        if (rss_table.size() == WINDOWSIZE) {
            return true;
        }
        return false;
    }

    public void clearRSSTable() {
        this.rss_table.clear();
    }

    public int getRSS() {
        avg_rss = 0;
        int max = -9999;
        int min = 9999;

        for (int i : rss_table) {
            avg_rss += i;
            if (max < i) max = i;
            if (min > i) min = i;
        }

        avg_rss -= (max + min);
        avg_rss /= (WINDOWSIZE - 2);

        return avg_rss;
    }

    public int getRawRSS() {
        return raw_rss;
    }
}
