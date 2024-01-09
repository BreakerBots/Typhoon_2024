// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites.drivetrain.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.LegacyBreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.test.suites.BreakerTestBase;
import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

/** Tests the maximum achiveable free velocites of each swerve module's drive motors */
public class BreakerSwerveModuleVelocityLimitTest extends BreakerTestBase{
    private double maxVelocityToTest;
    private final Timer timer = new Timer();
    private LegacyBreakerSwerveDrive drivetrain;
    private BreakerGenericSwerveModule[] swerveModules;
    private double[] maxVels;
    private double timeoutSeconds;
    public BreakerSwerveModuleVelocityLimitTest(LegacyBreakerSwerveDrive drivetrain, BreakerGenericSwerveModule[] swerveModules, double maxVelocityToTest, double timeoutSeconds, BreakerTestSuiteDataLogType logType) {
        super(logType, " Swerve_Module_Velocity_Limit_Test ", " Max_velocity_to_test: " + maxVelocityToTest);
        maxVels = new double[swerveModules.length];
        this.maxVelocityToTest = maxVelocityToTest;
        this.swerveModules = swerveModules;
        this.drivetrain = drivetrain;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(drivetrain);
    }

    public BreakerSwerveModuleVelocityLimitTestResult getResult() {
        return new BreakerSwerveModuleVelocityLimitTestResult(maxVelocityToTest, timeoutSeconds, swerveModules, maxVels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      timer.start();
      timer.reset();
    }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    StringBuilder work = new StringBuilder();
    work.append("Module States (Target Speed: " + maxVelocityToTest + ") - ");
    for (int i = 0; i < swerveModules.length; i++) {
        BreakerGenericSwerveModule mod = swerveModules[i];
        mod.setModuleTarget(new Rotation2d(), maxVelocityToTest, true);
        work.append(" ("+mod.getDeviceName() + " | current speed: " + mod.getModuleVelMetersPerSec() + ") ");
        double curSpeed = mod.getModuleVelMetersPerSec();
        if (maxVels[i] < curSpeed) {
            maxVels[i] = curSpeed;
        }
    }
    periodicLog(work.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logEnd(getResult().toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= timeoutSeconds;
  }

  public static class BreakerSwerveModuleVelocityLimitTestResult {
    private double targetSpeed;
    private double testTimeout;
    private double[] maxSpeeds;
    private BreakerGenericSwerveModule[] swerveModules;
    public BreakerSwerveModuleVelocityLimitTestResult(double targetSpeed, double testTimeout, BreakerGenericSwerveModule[] swerveModules, double... maxSpeeds) {
        this.targetSpeed = targetSpeed;
        this.testTimeout = testTimeout;
        this.maxSpeeds = maxSpeeds;
        this.swerveModules = swerveModules;
    }

    public double[] getMaxSpeeds() {
        return maxSpeeds;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public double getTestTimeout() {
        return testTimeout;
    }

    @Override
    public String toString() {
        StringBuilder work = new StringBuilder();
        for (int i = 0; i < maxSpeeds.length; i++) {
            work.append(String.format(" (Module Name: %s, Max Speed: %.2f) ", swerveModules[i].getDeviceName(), maxSpeeds[i]));
        }
        return String.format("Target Speed: %.2f | Test Timeout Seconds: %s | Max Speeds: %s", targetSpeed, testTimeout, work.toString());
    }
  }


}


