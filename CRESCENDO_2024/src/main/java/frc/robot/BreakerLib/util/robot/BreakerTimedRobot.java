// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.BreakerLib.util.robot;

import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/**
 * LoggedRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>
 * The LoggedRobot class is intended to be subclassed by a user creating a robot
 * program, and will call all required AdvantageKit periodic methods.
 *
 * <p>
 * periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
public class BreakerTimedRobot extends LoggedRobot {

    protected BreakerTimedRobot() {
        this(defaultPeriodSecs);
    }
    
      /**
       * Constructor for LoggedRobot.
       *
       * @param period Period in seconds.
       */
      protected BreakerTimedRobot(double period) {
        super(period);
        BreakerLog.disableDeterministicTimestamps();
      }

    @Override
  protected void loopFunc() {
    super.loopFunc();
    BreakerLog.captureAndProcessLoggables();
  }
}