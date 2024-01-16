// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerSwerveCANcoder implements BreakerSwerveAzimuthEncoder {
    private CANcoder encoder;
    private BreakerSwerveAzimuthEncoderSimIO simIO;
    public BreakerSwerveCANcoder(CANcoder encoder) {
        this.encoder = encoder;
        //CANcoderSimState simState = encoder.getSimState();
         
        // simIO = new BreakerSwerveAzimuthEncoderSimIO(
        //     simState::setRawPosition,
        //     simState::setVelocity,
        //     simState::setRawPosition,
        //     (boolean invert) -> {simState.Orientation = invert ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;}
        // );
    }

    @Override
    public double getRelative() {
        return BaseStatusSignal.getLatencyCompensatedValue(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public double getAbsolute() {
        return MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(encoder.getAbsolutePosition(), encoder.getVelocity()), -0.5, 0.5);
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return BreakerPhoenix6Util.checkCANcoderFaultsAndConnection(encoder);
    }

    @Override
    public Object getBaseEncoder() {
        return encoder;
    }

    // @Override
    // public BreakerSwerveAzimuthEncoderSimIO getSimIO() {
    //     return simIO;
    // }

    @Override
    public void config(boolean invertEncoder, double offset) {
        BreakerCANCoderFactory.configExistingCANCoder(encoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, offset, invertEncoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
        simIO.setSimAngle(0.0);
        simIO.setSimAngleVel(0.0);
        simIO.setInverted(invertEncoder);
        simIO.setSimSupplyVoltage(12.0);
    }

    @Override
    public void setStatusUpdatePeriod(double period) {
        BaseStatusSignal.setUpdateFrequencyForAll(1.0/period, encoder.getAbsolutePosition(), encoder.getPosition(), encoder.getVelocity());
    }

}