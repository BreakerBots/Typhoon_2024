// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/** A base class extended by all tests found in a given system's "TestSuite" implamentation */
public class BreakerTestBase extends Command {
    private BreakerTestSuiteDataLogType logType;
    private String startString;
    private String testName;
    public BreakerTestBase(BreakerTestSuiteDataLogType logType, String testName, String testInfo) {
        this.logType = logType;
        this.testName = testName;
        startString = "NEW TEST STARTED ( " + testName + " ): " + testInfo;
    }

    public void logStart() {
        BreakerLog.logEvent(startString);
    }

    public void logEnd(String results) {
        BreakerLog.logEvent("TEST ( " + testName + " ) ENDED AT: (T+) " + Timer.getFPGATimestamp());
        if (logType != BreakerTestSuiteDataLogType.NONE) {
            BreakerLog.logMessage("RESULTS FOR ( " + testName + " ): " + results);
        }
    }

    public void periodicLog(String message) {
        if (logType == BreakerTestSuiteDataLogType.LIVE_AUTOLOG) {
            System.out.println(message);
        }
    }

    public BreakerTestSuiteDataLogType getLogType() {
        return logType;
    }
}
