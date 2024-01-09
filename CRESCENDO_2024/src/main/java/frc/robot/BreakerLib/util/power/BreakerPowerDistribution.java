// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.power;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionJNI;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;

/**
 * Class for getting voltage, current, temperature, power and energy from the CTRE Power
 * Distribution Panel (PDP) or REV Power Distribution Hub (PDH) over CAN.
 */
public class BreakerPowerDistribution extends BreakerGenericDevice implements Sendable, AutoCloseable {
  private static BreakerPowerDistribution instance;
  private final int handle;
  private final int module;

  public static final int kDefaultModule = PowerDistributionJNI.DEFAULT_MODULE;

  public enum ModuleType {
    kCTRE(PowerDistributionJNI.CTRE_TYPE),
    kRev(PowerDistributionJNI.REV_TYPE);

    public final int value;

    ModuleType(int value) {
      this.value = value;
    }
  }

  /**
   * Constructs a PowerDistribution object.
   *
   * @param module The CAN ID of the PDP/PDH.
   * @param moduleType Module type (CTRE or REV).
   */
  private BreakerPowerDistribution(int module, edu.wpi.first.wpilibj.PowerDistribution.ModuleType moduleType) {
    this.handle = PowerDistributionJNI.initialize(module, moduleType.value);
    this.module = PowerDistributionJNI.getModuleNumber(handle);

    HAL.report(tResourceType.kResourceType_PDP, module + 1);
    SendableRegistry.addLW(this, "PowerDistribution", module);
  }

  /**
   * Constructs a PowerDistribution object.
   *
   * <p>Detects the connected PDP/PDH using the default CAN ID (0 for CTRE and 1 for REV).
   */
  private BreakerPowerDistribution() {
    this.handle = PowerDistributionJNI.initialize(kDefaultModule, PowerDistributionJNI.AUTOMATIC_TYPE);
    this.module = PowerDistributionJNI.getModuleNumber(handle);

    HAL.report(tResourceType.kResourceType_PDP, module + 1);
    SendableRegistry.addLW(this, "PowerDistribution", module);
  }

  public static BreakerPowerDistribution getInstance() {
    if (instance == null) {
      instance = new BreakerPowerDistribution();
    }
    return instance;
  }

  public static BreakerPowerDistribution getInstance(int moduleID, PowerDistribution.ModuleType moduleType) {
    if (instance == null) {
      instance = new BreakerPowerDistribution(moduleID, moduleType);
    } else if (instance.getModule() != moduleID || instance.getType().value != moduleType.value) {
      instance = new BreakerPowerDistribution(moduleID, moduleType);
    }

    return instance;
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Gets the number of channels for this power distribution object.
   *
   * @return Number of output channels (16 for PDP, 24 for PDH).
   */
  public int getNumChannels() {
    return PowerDistributionJNI.getNumChannels(handle);
  }

  /**
   * Query the input voltage of the PDP/PDH.
   *
   * @return The voltage in volts
   */
  public double getVoltage() {
    return PowerDistributionJNI.getVoltage(handle);
  }

  /**
   * Query the temperature of the PDP/PDH.
   *
   * @return The temperature in degrees Celsius
   */
  public double getTemperature() {
    return PowerDistributionJNI.getTemperature(handle);
  }

  /**
   * Query the current of a single channel of the PDP/PDH.
   *
   * @param channel The channel (0-15 for PDP, 0-23 for PDH) to query
   * @return The current of the channel in Amperes
   */
  public double getCurrent(int channel) {
    double current = PowerDistributionJNI.getChannelCurrent(handle, channel);

    return current;
  }

  /**
   * Query the current of all monitored channels.
   *
   * @return The current of all the channels in Amperes
   */
  public double getTotalCurrent() {
    return PowerDistributionJNI.getTotalCurrent(handle);
  }

  /**
   * Query the total power drawn from the monitored channels.
   *
   * @return the total power in Watts
   */
  public double getTotalPower() {
    return PowerDistributionJNI.getTotalPower(handle);
  }

  /**
   * Query the total energy drawn from the monitored channels.
   *
   * @return the total energy in Joules
   */
  public double getTotalEnergy() {
    return PowerDistributionJNI.getTotalEnergy(handle);
  }

  /** Reset the total energy to 0. */
  public void resetTotalEnergy() {
    PowerDistributionJNI.resetTotalEnergy(handle);
  }

  /** Clear all PDP/PDH sticky faults. */
  public void clearStickyFaults() {
    PowerDistributionJNI.clearStickyFaults(handle);
  }

  /**
   * Gets module number (CAN ID).
   *
   * @return The module number (CAN ID).
   */
  public int getModule() {
    return module;
  }

  /**
   * Gets the module type for this power distribution object.
   *
   * @return The module type
   */
  public ModuleType getType() {
    int type = PowerDistributionJNI.getType(handle);
    if (type == PowerDistributionJNI.REV_TYPE) {
      return ModuleType.kRev;
    } else {
      return ModuleType.kCTRE;
    }
  }

  /**
   * Gets whether the PDH switchable channel is turned on or off. Returns false with the CTRE PDP.
   *
   * @return The output state of the PDH switchable channel
   */
  public boolean getSwitchableChannel() {
    return PowerDistributionJNI.getSwitchableChannel(handle);
  }

  /**
   * Sets the PDH switchable channel on or off. Does nothing with the CTRE PDP.
   *
   * @param enabled Whether to turn the PDH switchable channel on or off
   */
  public void setSwitchableChannel(boolean enabled) {
    PowerDistributionJNI.setSwitchableChannel(handle, enabled);
  }

  public PowerDistributionVersion getVersion() {
    return PowerDistributionJNI.getVersion(handle);
  }

  public PowerDistributionFaults getPowerDistributionFaults() {
    return PowerDistributionJNI.getFaults(handle);
  }

  public PowerDistributionStickyFaults getStickyFaults() {
    return PowerDistributionJNI.getStickyFaults(handle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PowerDistribution");
    int numChannels = getNumChannels();
    for (int i = 0; i < numChannels; ++i) {
      final int chan = i;
      builder.addDoubleProperty(
          "Chan" + i, () -> PowerDistributionJNI.getChannelCurrentNoError(handle, chan), null);
    }
    builder.addDoubleProperty(
        "Voltage", () -> PowerDistributionJNI.getVoltageNoError(handle), null);
    builder.addDoubleProperty(
        "TotalCurrent", () -> PowerDistributionJNI.getTotalCurrent(handle), null);
    builder.addBooleanProperty(
        "SwitchableChannel",
        () -> PowerDistributionJNI.getSwitchableChannelNoError(handle),
        value -> PowerDistributionJNI.setSwitchableChannel(handle, value));
  }
    @Override
    public void runSelfTest() {
        
        
    }

}
