// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperstructureState;
import frc.robot.subsystems.Flywheel.FlywheelMode;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  private SuperstructureState superstructureState;
  private Intake intake;
  private Flywheel flywheel;
  private PastaRoller pastaRoller;
  private ShooterCarrage shooterCarrage;
  private Climb climb;
  private Drive drivetrain;
  public Superstructure() {}

  setSuperstructureState()

  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  @Override
  public void periodic() {
    pastaRoller.setState(superstructureState.getPastaRollerState());
    intake.setState(superstructureState.getIntakeState());
    shooterCarrage.setHopperState(superstructureState.getShooterCarrageHopperState());
    shooterCarrage.setPitchMode(superstructureState.getShooterCarragePitchMode());
    FlywheelMode desiredFlywheelMode = superstructureState.getFlywheelMode();
    if (shooterCarrage.hasNote() && desiredFlywheelMode == FlywheelMode.NEUTRAL) {
      desiredFlywheelMode = FlywheelMode.SHOOT_SPEAKER;
    }
    flywheel.setMode(desiredFlywheelMode);
  }

  
}
