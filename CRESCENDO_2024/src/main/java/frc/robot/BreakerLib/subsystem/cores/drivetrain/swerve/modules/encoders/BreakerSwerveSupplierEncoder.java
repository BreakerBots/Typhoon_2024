// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerSwerveSupplierEncoder implements BreakerSwerveAzimuthEncoder {
    private DoubleSupplier relativeSupplier, absoluteSupplier;
    private double offset;
    private int invert;
    public BreakerSwerveSupplierEncoder(DoubleSupplier relativeSupplier, DoubleSupplier absoluteSupplier) {
        this.relativeSupplier = relativeSupplier;
        this.absoluteSupplier = absoluteSupplier;
    }

    public BreakerSwerveSupplierEncoder(DoubleSupplier relativeSupplier) {
        this.relativeSupplier = relativeSupplier;
        absoluteSupplier = () -> {return MathUtil.inputModulus(relativeSupplier.getAsDouble(), -0.5, 0.5);};
    }
 
    @Override
    public double getRelative() {
        return (invert * relativeSupplier.getAsDouble()) + offset;
    }

    @Override
    public double getAbsolute() {
        return (invert * absoluteSupplier.getAsDouble()) + offset;
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        invert = invertEncoder ? -1 : 1;
        this.offset = offset;
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return new Pair<DeviceHealth,String>(DeviceHealth.NOMINAL, "");
    }
    
    @Override
    public Object getBaseEncoder() {
        return relativeSupplier;
    }
    
}
