// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;
import frc.robot.SuperstructureState;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.subsystems.Flywheel.FlywheelPresisionType;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.ShooterCarrage.CarragePitchMode;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  private SuperstructureState superstructureState;

  private Intake intake;
  private Flywheel flywheel;
  private PastaRoller pastaRoller;
  private ShooterCarrage shooterCarrage;
  private Climb climb;
  private Drive drivetrain;
  private FireingSolution latestSpeakerFireingSolution;

  private final ShooterTarget SPEAKER_TARGET = new ShooterTarget(drivetrain, new Translation3d(), null);
  
  public Superstructure() {}

  public void setSuperstructureState(SuperstructureState stateRequest) {
    superstructureState = stateRequest;
  }

  /** This is a sligly oversensitive call that is true if a sensor detects a note in the robot or the robot is in a state in which it may have a note but woud not detect it */
  public boolean hasNote() {
    return
      pastaRoller.hasNote() || 
      shooterCarrage.hasNote() || 
      superstructureState == SuperstructureState.HANDOFF_TO_PASTA_ROLLER || 
      superstructureState == SuperstructureState.EXTAKE_FROM_PASTA_ROLLER ||
      superstructureState == SuperstructureState.EJECT_NOTE ||
      superstructureState == SuperstructureState.SHOOT_INTO_SPEAKER;
  }

  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  public boolean isAtTargetState() {
    return intake.isAtTargetState() && shooterCarrage.isAtTargetState() && flywheel.isAtTargetVelocity();
  }

  private void manageFlywheel(FlywheelState desiredFlywheelState) {
    switch (desiredFlywheelState) {
      case EJECT_NOTE:
      case INTAKE:
      case NEUTRAL:
      case PASTA_ROLLER_HANDOFF:
        flywheel.setTargetState(desiredFlywheelState.getRequiredVelocityRPS().get(), desiredFlywheelState.getRequiredPresision());
        break;
      case IDLE:
        flywheel.setTargetState(latestSpeakerFireingSolution.fireingVec().getMagnitude() * 0.75, desiredFlywheelState.getRequiredPresision());
        break;
      case SHOOT_SPEAKER:
        flywheel.setTargetState(latestSpeakerFireingSolution.fireingVec().getMagnitude(), desiredFlywheelState.getRequiredPresision());
        break;
    }
  }

  private void manageShooterCarragePitch(CarragePitchMode desiredPitchMode) {
    switch (desiredPitchMode) {
      case EJECT_NOTE:
      case STOW:
      case INTAKE_FROM_HUMAN_PLAYER:
      case PASTA_ROLLER_HANDOFF:
        shooterCarrage.setTargetPitch(desiredPitchMode.getTargetAngle().get());
        break;
      case HOLD_ARBITRARY:
        shooterCarrage.setTargetPitch(shooterCarrage.getPitch());
        break;
      case SHOOT_SPEAKER:
        shooterCarrage.setTargetPitch(latestSpeakerFireingSolution.fireingVec().getVectorRotation());
        break;    
    }
  }

  @Override
  public void periodic() {
    latestSpeakerFireingSolution = SPEAKER_TARGET.getFireingSolution();

    if (RobotState.isDisabled() || RobotState.isEStopped()) {
      superstructureState = SuperstructureState.ROBOT_NEUTRAL;
    }

    pastaRoller.setState(superstructureState.getPastaRollerState());
    intake.setState(superstructureState.getIntakeState());
    shooterCarrage.setHopperState(superstructureState.getShooterCarrageHopperState());
    manageShooterCarragePitch(superstructureState.getShooterCarragePitchMode());
    FlywheelState desiredFlywheelState = superstructureState.getFlywheelState();
    if (shooterCarrage.hasNote() && desiredFlywheelState == FlywheelState.NEUTRAL) {
      desiredFlywheelState = FlywheelState.IDLE;
    }
    manageFlywheel(desiredFlywheelState);
  }

  
}
