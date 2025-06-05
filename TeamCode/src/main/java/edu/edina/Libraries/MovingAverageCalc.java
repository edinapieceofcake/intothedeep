package edu.edina.Libraries;

import java.util.ArrayList;

public class MovingAverageCalc {
    private final double lookbackWindow;
    private final ArrayList<Double> tList, xList;
    private double average;

    public MovingAverageCalc(double lookbackWindow) {
        this.lookbackWindow = lookbackWindow;
        tList = new ArrayList<>();
        xList = new ArrayList<>();
    }

    public double getAverage() {
        return average;
    }

    public void update(double t, double x) {
        while (!tList.isEmpty()) {
            if (t - tList.get(0) > lookbackWindow) {
                tList.remove(0);
                xList.remove(0);
            } else {
                break;
            }
        }

        tList.add(t);
        xList.add(x);

        average = calcAverage(tList, xList);
    }

    public static double calcAverage(ArrayList<Double> tList, ArrayList<Double> xList) {
        if (tList.size() == 1)
            return xList.get(0);

        int n = tList.size();
        double sum = 0.0;
        for (int i = 0; i + 1 < n; i++)
            sum += 0.5 * (xList.get(i + 1) + xList.get(i)) * (tList.get(i + 1) - tList.get(i));

        return sum / (tList.get(n - 1) - tList.get(0));
    }
}
