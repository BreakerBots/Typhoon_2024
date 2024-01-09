// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakerBeamBreak extends SubsystemBase {
  /** Creates a new BreakerBeamBreak. */
  private DigitalInput input;
  private boolean brokenOnTrue, prevRead, hasChanged;
  private final Timer timeSinceLastChange = new Timer();
  public BreakerBeamBreak(int imputPortDIO, boolean brokenOnTrue) {
    input = new DigitalInput(imputPortDIO);
    prevRead = !brokenOnTrue;
    hasChanged = false;
  }

  public boolean isBroken() {
    return brokenOnTrue ? input.get() : !input.get();
  }

  public boolean hasChanged() {
    return hasChanged;
  }

  public double getTimeSinceLastChange() {
      return timeSinceLastChange.get();
  }

  @Override
  public void periodic() {
    hasChanged = prevRead != isBroken();
    if (hasChanged) {
      timeSinceLastChange.reset();
      timeSinceLastChange.start();
    }
  }
}
