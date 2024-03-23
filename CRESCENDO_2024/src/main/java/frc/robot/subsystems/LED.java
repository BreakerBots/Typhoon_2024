// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin.FixedPalettePattern;

public class LED extends SubsystemBase {
  public enum LEDState {
    INTAKING_FOR_SHOOTER(FixedPalettePattern.BREATH_GRAY),
    INTAKING_FOR_AMP(FixedPalettePattern.BREATH_GRAY),
    EXTAKING(FixedPalettePattern.HEARTBEAT_BLUE),
    
    SHOOTING_TO_SPEAKER(FixedPalettePattern.FIRE_LARGE),
    SCORING_IN_AMP(FixedPalettePattern.STROBE_GOLD),
    PERSUING_NOTE(FixedPalettePattern.RAINBOW_GLITTER),
    
    ENABLED_WITH_NOTE(FixedPalettePattern.FIRE_MEDIUM),
    ENABLED(FixedPalettePattern.BREATH_BLUE), // should be 'default' state
    
    SPOOLING_FLYWHEEL(FixedPalettePattern.CONFETTI),

    ERROR(FixedPalettePattern.HEARBEAT_RED);

    private FixedPalettePattern pattern;


    private LEDState(FixedPalettePattern pattern) {
      this.pattern = pattern;
    }

    public FixedPalettePattern getFixedPalettePattern() {
      return pattern;
    }

  }
  
  private final BreakerRevBlinkin blinkin;
  private LEDState currentState;

  private Intake intake;

  /** Subsystem for managing LED lights on the robot using a state machine. */
  public LED(Intake intake) {
    blinkin = new BreakerRevBlinkin(0);
    currentState = LEDState.ENABLED;
    this.intake = intake; // does not need requirement
    updateBlinkin();
  }

  public Command setStateCommand(LEDState state) {
    return runOnce(() -> setState(state));
  }

  /**
   * Returns to the 'default' state, which is either {@code LEDState.ENABLED} or {@code LEDState.ENABLED_WITH_NOTE}
   * 
   * @return nothing
   */
  public Command returnToRestState() {
    return runOnce(() -> {
      if (intake.hasNote()) {
        setState(LEDState.ENABLED_WITH_NOTE);
      } else {
        setState(LEDState.ENABLED);
      }
    });
  }

  public void setState(LEDState state) {
    currentState = state;
    updateBlinkin();
  }

  private void updateBlinkin() {
    blinkin.setFixedPalettePattern(currentState.getFixedPalettePattern());
  }

  public LEDState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
