// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Replacement for WPILib's SubsystemBase class for breakerlib internal classes */
public abstract class BreakerGenericLoopedDevice extends BreakerGenericDevice implements Subsystem, Sendable {
    private boolean telemetryEnabled;
        public BreakerGenericLoopedDevice() {
            String name = this.getClass().getSimpleName();
            name = name.substring(name.lastIndexOf('.') + 1);
            SendableRegistry.addLW(this, name, name);
            CommandScheduler.getInstance().registerSubsystem(this);
            telemetryEnabled = false;
        }
        
        /**
        * Gets the name of this Subsystem.
        *
        * @return Name
        */
        public String getName() {
           return SendableRegistry.getName(this);
        }
        
        /**
        * Sets the name of this Subsystem.
        *
        * @param name name
        */
        public void setName(String name) {
            SendableRegistry.setName(this, name);
        }
        
        /**
        * Gets the subsystem name of this Subsystem.
        *
        * @return Subsystem name
        */
        public String getSubsystem() {
            return SendableRegistry.getSubsystem(this);
        }
        
        /**
        * Sets the subsystem name of this Subsystem.
        *
        * @param subsystem subsystem name
        */
        public void setSubsystem(String subsystem) {
           SendableRegistry.setSubsystem(this, subsystem);
        }
        
        /**
        * Associates a {@link Sendable} with this Subsystem. Also update the child's name.
        *
        * @param name name to give child
        * @param child sendable
        */
        public void addChild(String name, Sendable child) {
            SendableRegistry.addLW(child, getSubsystem(), name);
        }
        
        
        /** 
         * @param builder
         */
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Subsystem");
        
            builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
            builder.addStringProperty(
                ".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                null);
            builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
            builder.addStringProperty(
                ".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                null);
        }
     
}
