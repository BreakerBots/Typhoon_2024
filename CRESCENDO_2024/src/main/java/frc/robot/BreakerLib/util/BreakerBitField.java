// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;

import com.fasterxml.jackson.databind.util.ArrayIterator;

/** A class that represents a modifiable bit field for user convienance */
public final class BreakerBitField implements java.lang.Iterable<Boolean>, java.lang.Cloneable {
    public final Boolean[] bools;

    public BreakerBitField(boolean... bools) {
        this.bools = new Boolean[bools.length];
        for (int i = 0; i < bools.length; i++) {
            this.bools[i] = bools[i];
        }
    }

    public BreakerBitField(Boolean... bools) {
        this.bools = Arrays.copyOf(bools, bools.length);
    }

    public BreakerBitField(Collection<Boolean> bools) {
        this(bools.toArray(new Boolean[bools.size()]));
    }

    public BreakerBitField(long bitField) {
        bools = new Boolean[64];
        long bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(int bitField) {
        bools = new Boolean[32];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(short bitField) {
        bools = new Boolean[16];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(byte bitField) {
        bools = new Boolean[8];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(boolean bit) {
        bools = new Boolean[]{bit};
    }

    public BreakerBitField(BigInteger bitField) {
        byte[] data = bitField.toByteArray();
        ArrayList<Boolean> bools = new ArrayList<>();
        for (byte b: data) {
            byte mask = 1;
            for (int i = 0; i < 8; i++) {
                if (bools.size() < Integer.MAX_VALUE) {
                    bools.add((b & mask) == 1);
                    mask <<= 1;
                }
            }
        }
        this.bools = bools.toArray(new Boolean[bools.size()]);
    }

    public BreakerBitField and(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] & outher.bools[i]);
            } else {
                newBools[i] = false;
            }  
        }
        return new BreakerBitField(newBools);
    }

    public BreakerBitField or(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] | outher.bools[i]);
            } else {
                if (bools.length > outher.bools.length) {
                    newBools[i] = bools[i];
                } else {
                    newBools[i] = outher.bools[i];
                }
            }  
        }
        return new BreakerBitField(newBools);
    }

    public BreakerBitField xor(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] ^ outher.bools[i]);
            } else {
                if (bools.length > outher.bools.length) {
                    newBools[i] = bools[i];
                } else {
                    newBools[i] = outher.bools[i];
                }
            }  
        }
        return new BreakerBitField(newBools);
    }

    public BreakerBitField BitwiseComplament() {
        boolean[] newBools = new boolean[bools.length];
        for (int i = 0; i < bools.length; i++) {
            newBools[i] = !bools[i];
        }
        return new BreakerBitField(bools);
    }

    public boolean get(int index) {
        return bools[index];
    }

    public int length() {
        return bools.length;
    }

    public BreakerBitField truncate(int startIndex, int endIndex) {
        ArrayList<Boolean> list = new ArrayList<>();
        for (int i = startIndex; i <= endIndex && i < bools.length; i++) {
            list.add(bools[i]);
        }
        return new BreakerBitField(list.toArray(new Boolean[list.size()]));
    }

    public Boolean[] getBaseBooleanArray() {
        return Arrays.copyOf(bools, bools.length);
    }

    @Override
    public Iterator<Boolean> iterator() {
        return new ArrayIterator<Boolean>(bools);
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        return (Object) (new BreakerBitField(bools));
    }
}
