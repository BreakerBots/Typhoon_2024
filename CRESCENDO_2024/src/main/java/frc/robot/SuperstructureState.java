// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.PastaRoller.PastaRollerState;
import frc.robot.subsystems.ShooterCarrage.CarrageHopperState;
import frc.robot.subsystems.ShooterCarrage.CarragePitchMode;

/** Add your docs here. */
public enum SuperstructureState {
    /*Engagued by a disable event to ensure te robot does not snap back to its pre-disabled state if any changes occur */
    ROBOT_NEUTRAL(IntakeState.NEUTRAL, PastaRollerState.NEUTRAL, CarrageHopperState.NEUTRAL, CarragePitchMode.HOLD_ARBITRARY),
    STOW(IntakeState.RETRACTED_NEUTRAL, PastaRollerState.NEUTRAL, CarrageHopperState.NEUTRAL, CarragePitchMode.STOW),
    
    INTAKE_EXTENDED_HOLD(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.NEUTRAL, CarrageHopperState.NEUTRAL, CarragePitchMode.STOW),
    INTAKE_FROM_GROUND(IntakeState.EXTENDED_INTAKEING, PastaRollerState.NEUTRAL, CarrageHopperState.NEUTRAL, CarragePitchMode.STOW),
    INTAKE_TO_SHOOTER_HANDOFF(IntakeState.EXTENDED_INTAKEING, PastaRollerState.NEUTRAL, CarrageHopperState.INTAKE, CarragePitchMode.STOW),
    SHOOTER_TO_INTAKE_HANDOFF(IntakeState.EXTENDED_EXTAKEING, PastaRollerState.NEUTRAL, CarrageHopperState.EXTAKE, CarragePitchMode.STOW),
    INTAKE_TO_PASTA_ROLLER_HANDOFF(IntakeState.RETRACTED_EXTAKEING, PastaRollerState.INTAKE, CarrageHopperState.NEUTRAL, CarragePitchMode.STOW),

    SHOOTER_HOLD_NOTE(IntakeState.NEUTRAL, PastaRollerState.NEUTRAL, CarrageHopperState.NEUTRAL, CarragePitchMode.TRACK_TARGET),
    SHOOT_TO_TARGET(IntakeState.NEUTRAL, PastaRollerState.NEUTRAL, CarrageHopperState.INTAKE, CarragePitchMode.TRACK_TARGET);

    
    final IntakeState intakeState; 
    final PastaRollerState pastaRollerState; 
    final CarragePitchMode shooterCarragePitchMode; 
    final CarrageHopperState shooterCarrageHopperState; 
    private SuperstructureState(IntakeState intakeState, PastaRollerState pastaRollerState, CarrageHopperState shooterCarrageHopperState, CarragePitchMode shooterCarragePitchMode) {
        this.intakeState = intakeState;
        this.shooterCarrageHopperState = shooterCarrageHopperState;
        this.shooterCarragePitchMode = shooterCarragePitchMode;
        this.pastaRollerState = pastaRollerState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public PastaRollerState getPastaRollerState() {
        return pastaRollerState;
    }

    public CarrageHopperState getShooterCarrageHopperState() {
        return shooterCarrageHopperState;
    }

    public CarragePitchMode getShooterCarragePitchMode() {
        return shooterCarragePitchMode;
    }


}
