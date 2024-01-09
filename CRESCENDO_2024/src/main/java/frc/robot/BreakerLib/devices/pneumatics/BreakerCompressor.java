// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;

/**
 * Compressor bundled with PCM/PH and support for an analog pressure sensor.
 */
public class BreakerCompressor extends BreakerGenericDevice {

    private PneumaticsModuleType moduleType;

    private PneumaticsBase pneumaticModule;
    private AnalogPotentiometer analogPressureSensor = new AnalogPotentiometer(0); // Placeholder
    private boolean usingExternalPressureSensor;

    /**
     * Creates a new BreakerCompressor. with provided ID and type.
     * 
     * @param moduleID   CAN ID for the module.
     * @param moduleType CTRE or REV.
     */
    public BreakerCompressor(int moduleID, PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        deviceName = "Pnumatics_Module";
        moduleSetup(moduleID);
    }

    /**
     * Creates a new BreakerCompressor with default module ID (0 for PCM, 1 for PH).
     * 
     * @param moduleType CTRE or REV.
     */
    public BreakerCompressor(PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        deviceName = "Pnumatics_Module";
        moduleSetup();
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup(int id) {
        switch (moduleType) {
            case CTREPCM:
                pneumaticModule = new PneumaticsControlModule(id); // Registered as specific model for self test
                break;
            case REVPH:
                pneumaticModule = new PneumaticHub(id); // Registered as specific model for self test
                break;
        }
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup() {
        switch (moduleType) {
            case CTREPCM:
                moduleSetup(0);
                break;
            case REVPH:
                moduleSetup(1);
                break;
        }
    }

    /**
     * Creates an analog pressure sensor based on the REV Analog Pressure Sensor.
     * <p>
     * Only need to do this if using PCM. Otherwise just plug the sensor into analog
     * port 0 on the PH.
     */
    public void addAnalogPressureSensor(int analog_channel) {
        addAnalogPressureSensor(analog_channel, 250, -25);

    }

    /**
     * Creates an analog pressure sensor with given full range and offset.
     * <p>
     * Only need to do this if using PCM. Otherwise just plug the sensor into analog
     * port 0 on the PH.
     * 
     * @param analog_channel Analog port the sensor is plugged into.
     * @param fullRange      Scaling multiplier to output (check with part
     *                       manufacturer).
     * @param offset         Offset added to scaled value.
     */
    public void addAnalogPressureSensor(int analog_channel, double fullRange, double offset) {
        analogPressureSensor = new AnalogPotentiometer(analog_channel, fullRange, offset);
        usingExternalPressureSensor = true;
    }

    /**
     * @return PSI measured by analog pressure sensor. If no pressure sensor,
     * returns 0.
     */
    public double getPressure() {
        switch (moduleType) {
            case REVPH:
                return pneumaticModule.getPressure(0);
            case CTREPCM:
                if (usingExternalPressureSensor) {
                    return analogPressureSensor.get();
                } else {
                    return 0.0;
                }
        }
        return 0;
    }

    /** @return Voltage of analog port 0 if supported. */
    public double getVoltage() {
        return pneumaticModule.getAnalogVoltage(0);
    }

    /** @return Current in amps used by the compressor */
    public double getCompressorAmps() {
        return pneumaticModule.getCompressorCurrent();
    }

    /** @return Base pneumatic module. */
    public PneumaticsBase getModule() {
        return pneumaticModule;
    }

    /** @return True if the compressor is enabled. */
    public boolean compressorIsEnabled() {
        return pneumaticModule.getCompressor();
    }

    /** Enables closed loop compressor control using digital input. */
    public void enableDigital() {
        pneumaticModule.enableCompressorDigital();
    }

    /**
     * Enables closed loop compressor control using analog input from REV Analog
     * Pressure Sensor only. Defaults to digital control if CTRE PCM.
     * 
     * @param minPressure Compressor enables when pressure is below this value.
     * @param maxPressure Compressor disables when pressure is above this value.
     */
    public void enableAnalog(double minPressure, double maxPressure) {
        pneumaticModule.enableCompressorAnalog(minPressure, maxPressure);
    }

    /**
     * Enables closed loop compressor control using hybrid input from REV Analog
     * Pressure Sensor only. Defaults to digital control if CTRE PCM.
     * 
     * @param minPressure Compressor enables when pressure is below this value.
     * @param maxPressure Compressor disables when pressure is above this value.
     */
    public void enableHybrid(double minPressure, double maxPressure) {
        pneumaticModule.enableCompressorHybrid(minPressure, maxPressure);
    }

    /** Disables the compressor, shutting it down. */
    public void disable() {
        pneumaticModule.disableCompressor();
    }

    @Override
    public void runSelfTest() {
        switch (moduleType) {
            case CTREPCM:
                break;
            case REVPH:
                break;

        }

    }

}
