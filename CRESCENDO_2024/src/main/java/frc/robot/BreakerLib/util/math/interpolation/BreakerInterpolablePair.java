// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import edu.wpi.first.math.Pair;

/** Add your docs here. */
public class BreakerInterpolablePair<K extends BreakerInterpolable<K>, V extends BreakerInterpolable<V>> extends Pair<K, V> implements BreakerInterpolable<BreakerInterpolablePair<K,V>> {
    public BreakerInterpolablePair(K first, V second) {
        super(first, second);
    }

    @Override
    public BreakerInterpolablePair<K, V> interpolate(BreakerInterpolablePair<K, V> endValue, double t) {
        K interKey = getFirst().interpolate(endValue.getFirst(), t);
        V interVal = getSecond().interpolate(endValue.getSecond(), t);
        return new BreakerInterpolablePair<K,V>(interKey, interVal);
    }

    @Override
    public double[] getInterpolatableData() {
        double[] firstArr = getFirst().getInterpolatableData();
        double[] secondArr = getSecond().getInterpolatableData();
        double[] newArr = new double[firstArr.length + secondArr.length];
        int newArrIndex = 0;
        for (int i = 0; i < firstArr.length; i++) {
            newArr[newArrIndex] = secondArr[i];
            newArrIndex++;
        }
        for (int i = 0; i < firstArr.length; i++) {
            newArr[newArrIndex] = secondArr[i];
            newArrIndex++;
        }
        return newArr;
    }

    @Override
    public BreakerInterpolablePair<K, V> fromInterpolatableData(double[] interpolatableData) {
        int fLen = getFirst().getInterpolatableData().length;
        int sLen = getSecond().getInterpolatableData().length;
        double[] fArr = new double[fLen];
        double[] sArr = new double[sLen];
        for (int i = 0; i < interpolatableData.length; i++) {
            if (i < fLen) {
                fArr[i] = interpolatableData[i];
            } else {
                sArr[i - fLen] = interpolatableData[i];
            }
        } 
        K nKey = getFirst().fromInterpolatableData(fArr);
        V nVal = getSecond().fromInterpolatableData(sArr);
        return new BreakerInterpolablePair<K,V>(nKey, nVal); 
    }
}
