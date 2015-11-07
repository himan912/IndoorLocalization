package com.example.wireless.indoorlocalization;

import java.util.ArrayList;

/**
 * Created by wireless on 2015-10-30.
 */
public class Anchor2D extends Location2D {
    private final int MAX_WINDOWSIZE = 3;
    private ArrayList<Integer> rss_table = new ArrayList<>();
    private int avg_rss = 0;
    private int raw_rss = 0;

    private float P0;
    private float MU;

    public Anchor2D(double x, double y, float P0, float MU){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
    }

    public Anchor2D(float x, float y, float P0, float MU){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
    }

    public void setP0(float P0){
        this.P0 = P0;
    }

    public void addRSS(int r) {
        raw_rss = r;
        rss_table.add(raw_rss);
        if (rss_table.size() > MAX_WINDOWSIZE) {
            rss_table.remove(0);
        }
    }

    public boolean isCalibrated() {
        if (rss_table.size() >= MAX_WINDOWSIZE) {
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

        avg_rss /= rss_table.size();
        //avg_rss -= (max + min);
        //avg_rss /= (rss_table.size() - 2);

        return avg_rss;
    }

    public int getTableSize(){
        return rss_table.size();
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
