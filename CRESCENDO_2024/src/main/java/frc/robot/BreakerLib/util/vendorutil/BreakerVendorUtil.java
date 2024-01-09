// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.vendorutil;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * This class provides util methods for use across devices from multiple vendors
 * such as bit-field fault search methods and more.
 */
public class BreakerVendorUtil {

    /**
     * 
     * @param faultBitField               Bitfield of faults to examine.
     * @param fieldPlacesAndHealthEffects Map of bitfield places and related {@link DeviceHealth} effects.
     * 
     * @return Device health as {@link DeviceHealth} values.
     */
    public static DeviceHealth getDeviceFaultsAsHealth(long faultBitField,
            HashMap<Integer, DeviceHealth> fieldPlacesAndHealthEffects) {
        DeviceHealth health = DeviceHealth.NOMINAL;
        if (faultBitField != 0) {
            long fieldMask = 1; // masks all but selected bit
            for (int fieldPlace = 0; fieldPlace < fieldPlacesAndHealthEffects.size(); fieldPlace++) {
                // Checks for 1s in bitfield that signifies an error.
                if (((faultBitField & fieldMask) != 0) && fieldPlacesAndHealthEffects.containsKey(fieldPlace)) {
                    health = health != DeviceHealth.INOPERABLE ? fieldPlacesAndHealthEffects.get(fieldPlace) : health;
                }
                fieldMask <<= 1; // Scrolls to next bit.
            }
        }
        return health;
    }

    /**
     * 
     * @param faultBitField               Bitfield of faults to examine.
     * @param fieldPlacesHealthEffectsAndFaultMessages Map of bitfield places and related {@link DeviceHealth} effects, along with fault messages.
     * 
     * @return Device health as {@link DeviceHealth} values along with list of faults.
     */
    public static Pair<DeviceHealth, String> getDeviceHealthAndFaults(long faultBitField,
            HashMap<Integer, Pair<DeviceHealth, String>> fieldPlacesHealthEffectsAndFaultMessages) {
        StringBuilder work = new StringBuilder();
        DeviceHealth health = DeviceHealth.NOMINAL;
        if (faultBitField != 0) {
            long fieldMask = 1; // masks all but selected bit
            for (int fieldPlace = 0; fieldPlace < fieldPlacesHealthEffectsAndFaultMessages.size(); fieldPlace++) {
                if (((faultBitField & fieldMask) != 0)
                        && fieldPlacesHealthEffectsAndFaultMessages.containsKey(fieldPlace)) { // Checks for 1s in
                                                                                               // bitfield that
                                                                                               // signifies error
                    work.append(fieldPlacesHealthEffectsAndFaultMessages.get(fieldPlace).getSecond());
                    health = health != DeviceHealth.INOPERABLE
                            ? fieldPlacesHealthEffectsAndFaultMessages.get(fieldPlace).getFirst()
                            : health;
                }
                fieldMask <<= 1; // Scrolls to next bit.
            }
        }
        return new Pair<DeviceHealth, String>(health, work.toString());
    }

    /**
     * Gets device faults as string
     * 
     * @param faultBitField Device faults as bitfield
     * @param fieldPlacesAndFaultMessages Map of bitfield places and related fault messages.
     * @return All faults found as string. If no faults are found, "" is returned.
     */
    public static String getDeviceFaultsAsString(long faultBitField,
            HashMap<Integer, String> fieldPlacesAndFaultMessages) {
        StringBuilder work = new StringBuilder();
        if (faultBitField != 0) {
            long fieldMask = 1; // masks all but selected bit
            for (int fieldPlace = 0; fieldPlace < fieldPlacesAndFaultMessages.size(); fieldPlace++) {
                if (((faultBitField & fieldMask) != 0) && fieldPlacesAndFaultMessages.containsKey(fieldPlace)) { // Checks
                                                                                                                 // for
                                                                                                                 // 1s
                                                                                                                 // in
                                                                                                                 // bitfield
                                                                                                                 // that
                                                                                                                 // signifies
                                                                                                                 // error
                    work.append(fieldPlacesAndFaultMessages.get(fieldPlace));
                }
                fieldMask <<= 1; // Scrolls to next bit.
            }
        }
        return work.toString();
    }
}
