// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class AutoRotationOverrideManager {
    private Shooter shooter;
    public AutoRotationOverrideManager(Shooter shooter) {
        this.shooter = shooter;
        restoreDefaultOverrde();
    }

    public void overrideDefault(Supplier<Optional<Rotation2d>> rotationTargetOverride) {
        PPHolonomicDriveController.setRotationTargetOverride(rotationTargetOverride);
    }

    public void restoreDefaultOverrde() {
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);
    }

    private Optional<Rotation2d> getRotationOverride() {
        if (RobotState.isAutonomous() && shooter.hasNote()) {
            return Optional.of(shooter.getActiveTargetFireingSolution().yaw());
        }
        return Optional.empty();
    }
}

