// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.ltv;

import edu.wpi.first.math.controller.LTVUnicycleController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.LegacyBreakerSwerveDrive;

/** Add your docs here. */
public class BreakerLTVUnicycleSwerveAutoPathFollowerConfig {
    
    private LTVUnicycleController unicycleController;
    private LegacyBreakerSwerveDrive drivetrain;
    private BreakerGenericOdometer odometer;
    public BreakerLTVUnicycleSwerveAutoPathFollowerConfig(LegacyBreakerSwerveDrive drivetrain, LTVUnicycleController unicycleController) {
        this(drivetrain, drivetrain, unicycleController);
    }

    public BreakerLTVUnicycleSwerveAutoPathFollowerConfig(LegacyBreakerSwerveDrive drivetrain, BreakerGenericOdometer odometer, LTVUnicycleController unicycleController) {
        this.unicycleController = unicycleController;
        this.drivetrain = drivetrain;
        this.odometer = odometer;
    }

    public LTVUnicycleController getUnicycleController() {
        return unicycleController;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }

    public LegacyBreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }
}
