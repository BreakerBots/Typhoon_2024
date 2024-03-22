// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin;

public class LED extends SubsystemBase {
  public enum LEDState {
    INTAKING_FOR_SHOOTER,
    INTAKING_FOR_AMP,
    EXTAKING,
    
    SHOOTING_TO_SPEAKER,
    SCORING_IN_AMP,
    PERSUING_NOTE,
    
    ENABLED_WITH_NOTE,
    ENABLED,
    DISABLED,
    
    SPOOLING_FLYWHEEL,

    ERROR,

  }
  
  private final BreakerRevBlinkin blinkin;

  /** Creates a new LED. */
  public LED() {
    blinkin = new BreakerRevBlinkin(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
