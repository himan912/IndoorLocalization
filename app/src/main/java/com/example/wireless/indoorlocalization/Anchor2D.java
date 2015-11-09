package com.example.wireless.indoorlocalization;

import java.util.ArrayList;

/**
 * Created by wireless on 2015-10-30.
 */
public class Anchor2D extends Location2D {
    private final int MAX_WINDOWSIZE = 20;
    private final int SPIKE = 5;
    private ArrayList<Integer> rss_table = new ArrayList<>();
    private int avg_rss = 0;
    private int raw_rss = 0;
    private int pre_rss = 0;

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

    public void addRSS(int r) {
        pre_rss = raw_rss;
        raw_rss = r;

        if((rss_table.size() >= MAX_WINDOWSIZE) &&
                (r>pre_rss+SPIKE || r<pre_rss-SPIKE)){
            return;
        }

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

    public int getRSS() {
        avg_rss = 0;

        for (int i : rss_table) {
            avg_rss += i;
        }

        avg_rss /= rss_table.size();
        //avg_rss -= (max + min);
        //avg_rss /= (rss_table.size() - 2);

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
