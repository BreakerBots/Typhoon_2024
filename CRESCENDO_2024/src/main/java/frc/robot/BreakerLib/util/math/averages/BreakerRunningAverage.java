// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.averages;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/** Add your docs here. */
public class BreakerRunningAverage implements BreakerGenericAveragingList<Double>{
    private LinkedList<Double> list;
    private int movingAvrageWindow;
    public BreakerRunningAverage(int movingAvrageWindow) {
         list = new LinkedList<>();
         this.movingAvrageWindow = movingAvrageWindow;
    }

    @Override
    public void addValue(Double valueToAdd) {
        list.add(valueToAdd);
        if (list.size() > movingAvrageWindow) {
            list.remove();
        }
    }

    @Override
    public Double getAverage() {
        double total = 0;
        for (double val: list) {
            total += val;
        }
        return total /= list.size();
    }

    @Override
    public Double getAverageBetweenGivenIndexes(int startIndex, int stopIndex) {
        double total = 0;
        for (int i = startIndex; i <= stopIndex; i++) {
            total += list.get(i);
        }
        return total /= stopIndex - startIndex;
    }

    @Override
    public Double[] getAsArray() {
        return list.toArray(new Double[list.size()]);
    }

    @Override
    public void clear() {
        list.clear();
    }

    @Override
    public void removeValueAtGivenIndex(int index) {
        list.remove(index);
    }

    @Override
    public boolean removeGivenValue(Object value) {
        return list.remove(value);
    }

    @Override
    public List<Double> getBaseList() {
        return list;
    }
}
