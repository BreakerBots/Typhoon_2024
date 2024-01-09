// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/**
 * Core class of BreakerLib's SelfTest fuctionality, handling periodic passive
 * (unless user configured otherwise)
 * diagnostic self tests of all manually or automatically regeistered
 * {@link BreakerSelfTestable} compatible devices, or devices registered
 * through an instance of the {@link SystemDiagnostics} class.
 */
public class SelfTest extends SubsystemBase {
  private int cycleCount;
  private static String lastSystemCheck;
  private static List<BreakerSelfTestable> devices = new ArrayList<BreakerSelfTestable>();
  private static int cyclesbetweenPerSelfCecks = 250;
  private static boolean lastCheckPassed = true;
  private static boolean autoRegesterDevices = true;
  private static boolean selfTestEnabled = true;

  /** Configures an enables a SelfTest check cycle */
  public SelfTest(double secondsBetweenPeriodicSelfChecks) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.autoRegesterDevices = true;
  }

  /** Configures an enables a SelfTest check cycle */
  public SelfTest(double secondsBetweenPeriodicSelfChecks, boolean autoRegesterDevices) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.autoRegesterDevices = autoRegesterDevices;
  }

  /** Enables or dissables the periodic self test diagnstic check cycle */
  public static void setSelfTestEnabled(boolean isEnabled) {
    selfTestEnabled = isEnabled;
  }

  /** @return if the periodic self test diagnstic check cycle is enabled */
  public static boolean isSelfTestEnabled() {
    return selfTestEnabled;
  }

  /**
   * Automaticly adds a {@link BreakerSelfTestable} compatable device to the
   * SelfTest queue if automatic registration is enabled.
   * <br>
   * <br>
   * WARNING: should only be used in BreakerLib internal classes and is toggled
   * based on user config of selftest.
   */
  public static void autoRegisterDevice(BreakerSelfTestable device) {
    if (autoRegesterDevices) {
      devices.add(device);
    }
  }

  /**
   * Automaticly adds multipul {@link BreakerSelfTestable} compatable devices to
   * the SelfTest queue if automatic registration is enabled.
   * <br>
   * <br>
   * WARNING: should only be used in BreakerLib internal classes and is toggled
   * based on user config of selftest.
   */
  public static void autoRegisterDevices(BreakerSelfTestable... devices) {
    if (autoRegesterDevices) {
      addDevices(devices);
    }
  }

  /**
   * Maunualy adds a {@link BreakerSelfTestable} compatable device to the SelfTest
   * queue.
   * <br>
   * <br>
   * WARNING: If this is done to a device that has allready been added
   * (including automatic additions, if enabled), will be checked twice,
   * possably with signifcant runtime expence
   */
  public static void addDevice(BreakerSelfTestable device) {
    devices.add(device);
  }

  /**
   * Maunualy adds multipul {@link BreakerSelfTestable} compatable devices to the
   * SelfTest queue.
   * <br>
   * <br>
   * WARNING: If this is done to a device that has allready been added
   * (including automatic additions, if enabled), will be checked twice,
   * possably with signifcant runtime expence
   */
  public static void addDevices(BreakerSelfTestable... devicesToAdd) {
    for (BreakerSelfTestable div : devicesToAdd) {
      devices.add(div);
    }
  }

  /**
   * @return The logged fault string result of the most recent self check cycle
   */
  public static String getLastSelfCheck() {
    return lastSystemCheck;
  }

  /**
   * @return True if the last self check was passes without a fault, false
   *         outherwise
   */
  public static boolean getLastSelfCheckPassed() {
    return lastCheckPassed;
  }

  /**
   * @return True if the {@link SelfTest#autoRegisterDevice()} and
   *         {@link SelfTest#autoRegisterDevices()} are enabled by the user, flase
   *         outherwise
   */
  public static boolean getAutoRegesterDevicesIsEnabled() {
    return autoRegesterDevices;
  }

  /**
   * Method that runs this robots Self Check cycle, pereodicly called by this
   * classes pereodic method
   */
  public static void runSelfTest() {
    StringBuilder work = new StringBuilder("\n RUNNING SELF TEST: \n");
    List<BreakerSelfTestable> faultDevices = new ArrayList<BreakerSelfTestable>();
    for (BreakerSelfTestable device : devices) {
      device.runSelfTest();
      if (device.hasFault()) {
        faultDevices.add(device);
      }
    }
    if (!faultDevices.isEmpty()) {
      work.append(" SELF TEST FAILED - FAULTS FOUND: \n");
      lastCheckPassed = false;
      for (BreakerSelfTestable faultDiv : faultDevices) {
        work.append(" | " + faultDiv.getDeviceName() + "-" + faultDiv.getFaults() + " | ");
      }
    } else {
      work.append(" SELF TEST PASSED ");
      lastCheckPassed = true;
    }
    lastSystemCheck = work.toString();
    BreakerLog.getInstance().logMessage(lastSystemCheck);
  }

  @Override
  public void periodic() {
    if ((cycleCount++ % cyclesbetweenPerSelfCecks == 0) && selfTestEnabled) {
      runSelfTest();
    }
  }
}
