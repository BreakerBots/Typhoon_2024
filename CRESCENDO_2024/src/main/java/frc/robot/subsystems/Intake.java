// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {
  private TalonFX rollerMotor;  
  private TalonFX pivotLeft, pivotRight;
  private IntakeState targetState;
  private CANcoder piviotEncoder;
  private DutyCycleOut piviotDutyCycleControlRequest;
  private Follower pivotFollowerRequest;
  private BreakerBeamBreak beamBreak;
 

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID, Constants.GeneralConstants.DRIVE_CANIVORE_NAME);
    pivotLeft = new TalonFX(PIVOT_LEFT_ID, Constants.GeneralConstants.DRIVE_CANIVORE_NAME);
    pivotRight = new TalonFX(PIVOT_RIGHT_ID, Constants.GeneralConstants.DRIVE_CANIVORE_NAME);
    piviotEncoder = new CANcoder(PIVOT_ENCODER_ID, Constants.GeneralConstants.DRIVE_CANIVORE_NAME);
    
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotLeft.getConfigurator().apply(pivotConfig);
    pivotRight.getConfigurator().apply(pivotConfig);

    BreakerCANCoderFactory.configExistingCANCoder(piviotEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, PIVOT_ENCODER_OFFSET, SensorDirectionValue.CounterClockwise_Positive);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 130;
    rollerConfig.CurrentLimits.SupplyTimeThreshold = 3.5;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(rollerConfig);

    piviotDutyCycleControlRequest = new DutyCycleOut(0.0);
    pivotFollowerRequest = new Follower(PIVOT_LEFT_ID, true);
  }

  public Command setStateCommand(IntakeState stateToSet, boolean waitForSuccess) {
    return new FunctionalCommand(() -> {setState(stateToSet);}, ()-> {}, (Boolean interrupted) -> {}, () -> {return !waitForSuccess||isAtTargetState();}, this);
  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public IntakeState getState() {
    return targetState;
  }

  public boolean isAtTargetState() {
    return switch(targetState.getPivotState()) {
      case EXTENDED -> isExtendLimitTriggered();
      case RETRACTED -> isRetractLimitTriggered();
      default -> true;
    };
  }

  public void setState(IntakeState stateToSet) {
    targetState = stateToSet;
  }

  public static enum IntakeState {
    EXTENDED_EXTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.EXTAKEING),
    EXTENDED_INTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.INTAKEING),
    EXTENDED_NEUTRAL(IntakePivotState.EXTENDED, IntakeRollerState.NEUTRAL),
    RETRACTED_NEUTRAL(IntakePivotState.RETRACTED, IntakeRollerState.NEUTRAL),
    RETRACTED_EXTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.EXTAKEING),
    RETRACTED_INTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.INTAKEING),
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
    EXTENDED(0.1),
    RETRACTED(-0.15),
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
    INTAKEING(-0.8),
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

  public boolean isExtendLimitTriggered() {
    return piviotEncoder.getAbsolutePosition().getValue() <= PIVIOT_EXTENDED_THRESHOLD;
  }

  public boolean isRetractLimitTriggered() {
    return piviotEncoder.getAbsolutePosition().getValue() >= PIVIOT_RETRACTED_THRESHOLD;
  }



  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      if (isExtendLimitTriggered()) {
        setState(IntakeState.EXTENDED_NEUTRAL);
      } else { 
        setState(IntakeState.RETRACTED_NEUTRAL);
      }
    }

    piviotDutyCycleControlRequest.withOutput(targetState.getPivotState().getMotorDutyCycle());
    piviotDutyCycleControlRequest.withLimitForwardMotion(isExtendLimitTriggered());
    piviotDutyCycleControlRequest.withLimitReverseMotion(isRetractLimitTriggered());
    BreakerLog.recordOutput("EXT LIM", isExtendLimitTriggered());
    BreakerLog.recordOutput("RET LIM", isRetractLimitTriggered());
    pivotLeft.setControl(piviotDutyCycleControlRequest);
    pivotRight.setControl(pivotFollowerRequest);
    rollerMotor.set(targetState.getRollerState().getMotorDutyCycle());
    
  }
}
