// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.averages;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;

import edu.wpi.first.math.MathUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerWeightedAverage implements BreakerGenericAveragingList<Double> {
    private List<Double> vals;
    private List<Double> weights;
    public BreakerWeightedAverage() {
        vals = new ArrayList<>();
        weights = new ArrayList<>();
    }

    @Override
    public void addValue(Double valueToAdd) {
        addValue(valueToAdd, 1.0);
    }

    public Double addValue(Double valueToAdd, Double weight) {
        vals.add(valueToAdd);
        weights.add(MathUtil.clamp(weight, 0.0, 1.0));
        return getAverage();
    }

    @Override
    public Double getAverage() {
        return BreakerMath.getWeightedAvg(vals, weights);
    }
    @Override
    public Double getAverageBetweenGivenIndexes(int startIndex, int stopIndex) {
        return BreakerMath.getWeightedAvg(vals.subList(startIndex, stopIndex), weights.subList(startIndex, stopIndex));
    }
    @Override
    public Double[] getAsArray() {
        return vals.toArray(new Double[vals.size()]);
    }

    public Double[] getWeightsAsArray() {
        return weights.toArray(new Double[weights.size()]);
    }

    @Override
    public List<Double> getBaseList() {
        return new ArrayList<>(vals);
    }

    public List<Double> getWeightList() {
        return new ArrayList<>(weights);
    }
    @Override
    public void clear() {
        vals.clear();
        weights.clear();
    }
    @Override
    public void removeValueAtGivenIndex(int index) {
        vals.remove(index);
        weights.remove(index);
    }
    @Override
    public boolean removeGivenValue(Object value) {
        int index = vals.indexOf(value);
        if (index != -1) {
            weights.remove(index);
            vals.remove(value);
            return true;
        }
        return false;
    }
}
