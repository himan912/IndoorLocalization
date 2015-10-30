package com.example.wireless.indoorlocalization;

import java.util.ArrayList;

/**
 * Created by wireless on 2015-10-30.
 */
public class Anchor extends Location{
    protected final int WINDOWSIZE = 5;
    protected ArrayList<Integer> rss_table = new ArrayList<>();
    protected int avg_rss = 0;
    protected int raw_rss = 0;

    protected float P0;
    protected float MU;

    public Anchor(double x, double y, float P0, float MU){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
    }

    public Anchor(float x, float y, float P0, float MU){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
    }

    public void addRSS(int r) {
        raw_rss = r;
        rss_table.add(raw_rss);
        if (rss_table.size() > WINDOWSIZE) {
            rss_table.remove(0);
        }
    }

    public boolean isCalibrated() {
        if (rss_table.size() >= WINDOWSIZE) {
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
        avg_rss /= (rss_table.size() - 2);

        return avg_rss;
    }

    public float getP0(){
        return P0;
    }

    public float getMU(){
        return MU;
    }

    public int getRawRSS() {
        return raw_rss;
    }
}
