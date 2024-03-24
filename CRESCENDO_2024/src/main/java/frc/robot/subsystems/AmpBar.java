// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AmpBarConstants.ENCODER_OFFSET;
import static frc.robot.Constants.AmpBarConstants.EXTENDED_ANGLE_THRESHOLD;
import static frc.robot.Constants.AmpBarConstants.RETRACTED_ANGLE_THRESHOLD;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;

public class AmpBar extends SubsystemBase {
  public enum AmpBarState {
    RETRACTED(-0.2),
    EXTENDED(1.0),
    NEUTRAL(0.0);

    private double motorDutyCycle;

    private AmpBarState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
      return motorDutyCycle;
    }
  }

  private CANSparkFlex sparkFlex;
  private CANcoder canCoder;
  private AmpBarState currentState;
  private Supplier<Double> pivotAbsPosSupplier;
  private boolean prevThermalProtectionState;
  private final Timer thermalProtectionTimer = new Timer();
  
  public Command setStateCommand(AmpBarState desiredState, boolean waitToFinish) {
    return new FunctionalCommand(
      () -> {
        setState(desiredState);
      }, 
      () -> {}, 
      (i) -> {},
      () -> !waitToFinish || isAtTargetAngle(),
      this
    );
  }

  public void setState(AmpBarState state) {
    currentState = state;
  }

  public boolean isAtTargetAngle() {
    return switch (currentState) {
      case EXTENDED -> isExtendLimitTriggered();
      case RETRACTED -> isRetractLimitTriggered();
      case NEUTRAL -> true;
    };
  }

  public boolean isExtendLimitTriggered() {
    return getPivotPosition().getRotations() >= EXTENDED_ANGLE_THRESHOLD.getRotations();
  }

  public boolean isRetractLimitTriggered() {
    return getPivotPosition().getRotations() <= RETRACTED_ANGLE_THRESHOLD.getRotations();
  }


  public Rotation2d getPivotPosition() {
    return Rotation2d.fromRotations(pivotAbsPosSupplier.get());
  }

  /** Creates a new AmpBar. */
  public AmpBar() {
    sparkFlex = new CANSparkFlex(60, MotorType.kBrushless); // TOOD fill in device id
    sparkFlex.restoreFactoryDefaults();
    sparkFlex.setIdleMode(IdleMode.kBrake);
    sparkFlex.setInverted(false);
    sparkFlex.setSmartCurrentLimit(80, 40);
    sparkFlex.setSecondaryCurrentLimit(100, 50);
    sparkFlex.burnFlash();
    canCoder = BreakerCANCoderFactory.createCANCoder(35, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, ENCODER_OFFSET.getRotations(), SensorDirectionValue.Clockwise_Positive); //check sensor dir based on cancoder pos
    pivotAbsPosSupplier = canCoder.getAbsolutePosition().asSupplier();
    prevThermalProtectionState = false;
    currentState = AmpBarState.RETRACTED;
  }

  public boolean isInThermalProtection() {
    boolean overCutoff = sparkFlex.getMotorTemperature() >= 60.0;
    if (overCutoff) {
      prevThermalProtectionState = true;
      thermalProtectionTimer.stop();
      thermalProtectionTimer.reset();
      return true;
    } else if (!overCutoff && prevThermalProtectionState) {
      thermalProtectionTimer.restart();
      prevThermalProtectionState = false;
    } else if (!overCutoff && !prevThermalProtectionState && thermalProtectionTimer.hasElapsed(5.0)) {
      return false;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotState.isDisabled()) {
      if (isExtendLimitTriggered()) {
        setState(AmpBarState.EXTENDED);
      } else { 
        setState(AmpBarState.RETRACTED);
      }
    }

    boolean thermalProtection = isInThermalProtection();
    if ((!isAtTargetAngle() || currentState == AmpBarState.EXTENDED) && !thermalProtection) {
      sparkFlex.set(currentState.motorDutyCycle);
    }
    else {
      sparkFlex.set(0.0);
    }
  }
}
