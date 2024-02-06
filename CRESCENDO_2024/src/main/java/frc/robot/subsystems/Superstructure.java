// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShooterTarget;
import frc.robot.SuperstructureState;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.subsystems.ShooterCarrage.CarragePitchMode;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  private SuperstructureState superstructureState;

  private Intake intake;
  private Flywheel flywheel;
  private PastaRoller pastaRoller;
  private ShooterCarrage shooterCarrage;
  private ClimbArm climb;
  private Drive drivetrain;
  private FireingSolution latestFireingSolution;
  private ShooterTarget selectedShooterTarget;
  public Superstructure(ShooterTarget initalShooterTarget) {
    selectedShooterTarget = initalShooterTarget;
    latestFireingSolution = selectedShooterTarget.getFireingSolution();
  }

  public void setSuperstructureState(SuperstructureState stateRequest) {
    superstructureState = stateRequest;
  }

  public boolean strictHasNote() {
    return intake.hasNote() || shooterCarrage.hasNote() || pastaRoller.hasNote();
  }

  public boolean intakeHasNote() {
    return intake.hasNote();
  }

  public boolean shooterCarrageHasNote() {
    return shooterCarrage.hasNote();
  }

  public boolean pastaRollerHasNote() {
    return pastaRoller.hasNote();
  }

  

  /** This is a sligly oversensitive call that is true if a sensor detects a note in the robot or the robot is in a state in which it may have a note but woud not detect it */
  public boolean hasNote() {
    return strictHasNote();
  }

  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  public boolean isAtTargetState() {
    return intake.isAtTargetState() && shooterCarrage.isAtTargetState() && flywheel.isAtTargetVelocity();
  }

  public void setSelectedShooterTarget(ShooterTarget newTarget) {
    selectedShooterTarget = newTarget;
  }

  private void manageShooterCarragePitch(CarragePitchMode desiredPitchMode) {
    switch (desiredPitchMode) {
      case STOW:
        shooterCarrage.setTargetPitch(Constants.ShooterCarrageConstants.STOW_ANGLE);
        break;
      case HOLD_ARBITRARY:
        shooterCarrage.setTargetPitch(shooterCarrage.getPitch());
        break;
      case TRACK_TARGET:
        shooterCarrage.setTargetPitch(latestFireingSolution.fireingVec().getVectorRotation());
        break;    
    }
  }

  @Override
  public void periodic() {
    latestFireingSolution = selectedShooterTarget.getFireingSolution();

    if (RobotState.isDisabled() || RobotState.isEStopped()) {
      superstructureState = SuperstructureState.ROBOT_NEUTRAL;
    }

    if (superstructureState == SuperstructureState.INTAKE_FROM_GROUND && intake.hasNote()) {
      superstructureState = SuperstructureState.INTAKE_EXTENDED_HOLD;
    }

    if (superstructureState == SuperstructureState.INTAKE_TO_SHOOTER_HANDOFF && shooterCarrageHasNote()) {
      superstructureState = SuperstructureState.SHOOTER_HOLD_NOTE;
    }

    if (superstructureState == SuperstructureState.INTAKE_TO_PASTA_ROLLER_HANDOFF && pastaRollerHasNote()) {
      superstructureState = SuperstructureState.PASTA_ROLLER_HOLD_NOTE;
    }

    pushStateRequests();
  }

  private void pushStateRequests() {
    pastaRoller.setState(superstructureState.getPastaRollerState());
    intake.setState(superstructureState.getIntakeState());
    shooterCarrage.setHopperState(superstructureState.getShooterCarrageHopperState());
    manageShooterCarragePitch(superstructureState.getShooterCarragePitchMode());
    if (superstructureState != SuperstructureState.ROBOT_NEUTRAL) {
      flywheel.setTargetVelocity(latestFireingSolution.fireingVec().getMagnitude());
    } else {
      flywheel.stop();
    }
  }

  public static enum ControlledNoteLocation {
    SHOOTER_CARRAGE,
    PASTA_ROLLER,
    PASTA_ROLLER_EXTAKE,
    PASTA_ROLLER_HANDOFF,
    CARRAGE_EXTAKE;
  }

  
}
