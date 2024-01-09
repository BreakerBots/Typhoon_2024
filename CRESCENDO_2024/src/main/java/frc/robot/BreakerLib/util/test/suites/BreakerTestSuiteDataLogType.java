// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites;

/** Add your docs here. */
public enum BreakerTestSuiteDataLogType {
    /** Does not log or print any of the data recorded by the test */
    NONE,
    /** log and prints the test's start, end, and result info */
    PARTIAL_AUTOLOG,
    /** logs and prints not only results, start,and end, but also data live as it is recoreded */
    LIVE_AUTOLOG
}
