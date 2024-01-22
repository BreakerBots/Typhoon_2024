// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX rollerMotor;  
  private TalonFX pivotMotor;
  private IntakeState targetState;
  private BooleanSupplier piviotForwardLimit;
  private BooleanSupplier piviotReverseLimit;
 

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new WPI_TalonSRX(ROLLER_MOTOR_ID);
    pivotMotor = new TalonFX(PIVOT_MOTOR_ID);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    pivotConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = true;

    pivotConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    pivotConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotMotor.getConfigurator().apply(pivotConfig);

    TalonSRXConfiguration rollerConfig = new TalonSRXConfiguration();
    rollerConfig.peakCurrentLimit = 60;
    rollerConfig.peakCurrentDuration = 1500;
    rollerConfig.continuousCurrentLimit = 20;


    Supplier<ForwardLimitValue> fwdLimSup = pivotMotor.getForwardLimit().asSupplier();
    piviotForwardLimit = () -> {return fwdLimSup.get() == ForwardLimitValue.ClosedToGround;};

    Supplier<ReverseLimitValue> revLimSup = pivotMotor.getReverseLimit().asSupplier();
    piviotReverseLimit = () -> {return revLimSup.get() == ReverseLimitValue.ClosedToGround;};
  }

  public IntakeState getState() {
    return targetState;
  }

  public boolean isAtTargetState() {
    if (targetState.getPivotState() == IntakePivotState.EXTENDED) {
      return piviotForwardLimit.getAsBoolean();
    }
    return piviotReverseLimit.getAsBoolean();
  }

  public void setState(IntakeState stateToSet) {
    targetState = stateToSet;
  }

  public static enum IntakeState {
    EXTENDED_INTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.INTAKEING),
    EXTENDED_NEUTRAL(IntakePivotState.EXTENDED, IntakeRollerState.NEUTRAL),
    RETRACTED_NEUTRAL(IntakePivotState.RETRACTED, IntakeRollerState.NEUTRAL),
    NEUTRAL(IntakePivotState.NEUTRAL, IntakeRollerState.NEUTRAL);
    
    private IntakePivotState pivotState;
    private IntakeRollerState rollerState;
    private IntakeState(IntakePivotState piviotState, IntakeRollerState rollerState) {
      this.pivotState = piviotState;
      this.rollerState = rollerState;
    }

    public IntakePivotState getPivotState() {
        return pivotState;
    }

    public IntakeRollerState getRollerState() {
        return rollerState;
    }
  }

  public static enum IntakePivotState {
    EXTENDED(0.6),
    RETRACTED(-0.6),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakePivotState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  public static enum IntakeRollerState {
    INTAKEING(-0.5),
    EXTAKEING(0.7),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakeRollerState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }



  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      if (piviotForwardLimit.getAsBoolean()) {
        setState(IntakeState.EXTENDED_NEUTRAL);
      } else { 
        setState(IntakeState.RETRACTED_NEUTRAL);
      }
    }
    rollerMotor.set(targetState.getRollerState().getMotorDutyCycle());
    pivotMotor.set(targetState.getPivotState().getMotorDutyCycle());
  }
}
