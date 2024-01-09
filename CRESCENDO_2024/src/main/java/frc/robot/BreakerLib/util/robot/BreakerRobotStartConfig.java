// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/** Configures info in robot startup message. */
public class BreakerRobotStartConfig {
    private int teamNum;
    private String teamName;
    private BreakerRobotNameConfig robotNameConfig;
    private int robotYear;
    private String robotSoftwareVersion;
    private String authorNames;

    /**
     * Constructs a BreakerStartConfig. Info will appear in startup message.
     * 
     * @param teamNum FRC team number.
     * @param teamName FRC team name.
     * @param robotName Name of your robot.
     * @param robotYear Competition year your robot was made for.
     * @param robotSoftwareVersion Robot software version.
     * @param authorNames Names of code authors. 
     */
    public BreakerRobotStartConfig(int teamNum, String teamName, BreakerRobotNameConfig robotNameConfig, int robotYear, String robotSoftwareVersion, String authorNames) {
        this.teamNum = teamNum;
        this.teamName = teamName;
        this.robotNameConfig = robotNameConfig;
        this.robotYear = robotYear;
        this.robotSoftwareVersion = robotSoftwareVersion;
        this.authorNames = authorNames;
    }

    /** @return Code author names. */
    public String getAuthorNames() {
        return authorNames;
    }

    /** @return Name of robot. */
    public String getRobotName() {
        return robotNameConfig.getRobotName();
    }

    /** @return Robot software version. */
    public String getRobotSoftwareVersion() {
        return robotSoftwareVersion;
    }

    /** @return Robot's game year. */
    public int getRobotYear() {
        return robotYear;
    }

    /** @return Team name. */
    public String getTeamName() {
        return teamName;
    }

    /** @return Team number. */
    public int getTeamNum() {
        return teamNum;
    }

    public static class BreakerRobotNameConfig {
        private Map<String, String> robotControllerSerialNumbersAndRobotNames;
        private String defaultName = "UNKNOWN ROBOT";
        public BreakerRobotNameConfig(Map<String, String> robotControllerSerialNumbersAndRobotNames) {
            this.robotControllerSerialNumbersAndRobotNames = robotControllerSerialNumbersAndRobotNames;
        }

        public BreakerRobotNameConfig(String robotName) {
            this(new HashMap<>());
            setDefaultName(robotName);
        }

        public BreakerRobotNameConfig() {
            this(new HashMap<>());
        }

        public BreakerRobotNameConfig addRobot(String robotControllerSerialNumber, String robotName) {
            robotControllerSerialNumbersAndRobotNames.put(robotControllerSerialNumber, robotName);
            return this;
        }

        public void setDefaultName(String defaultName) {
            this.defaultName = defaultName;
        }

        public String getRobotName(String serialNumber) {
            if (robotControllerSerialNumbersAndRobotNames.containsKey(serialNumber)) {
                return robotControllerSerialNumbersAndRobotNames.get(serialNumber);
            } else if (RobotBase.isSimulation()) {
                return "SIMULATION";
            }
            return defaultName + " | " + serialNumber; 
        }

        public String getRobotName() {
            return getRobotName(RobotController.getSerialNumber());
        }
    }

}
