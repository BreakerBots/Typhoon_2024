// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging.advantagekit;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.BuildConstants;
import frc.robot.BreakerLib.util.BreakerLibVersion;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import us.hebi.quickbuf.ProtoMessage;

/** Add your docs here. */
public class BreakerLog {
    private static Map<String, LoggableInputs> loggables = new HashMap<>();


    //NOTE: THE CODE BELOW IS NOT PRETTY, THAT IS BECAUSE IT IS A TEMPORARY WORKAROUND UNTILL Logger IS MADE FULLY STATIC

    public static void registerLogable(String name, LoggableInputs loggableInputs) {
        loggables.put(name, loggableInputs);
    }

    public static void captureAndProcessLoggables() {
        for (Entry<String, LoggableInputs> ent : loggables.entrySet()) {
            Logger.processInputs(ent.getKey(), ent.getValue());
        }
    }

    /**
   * Sets the source to use for replaying data. Use null to disable replay. This
   * method only works during setup before starting to log.
   */
  public static void setReplaySource(LogReplaySource replaySource) {
    Logger.setReplaySource(replaySource);
  }

  /**
   * Adds a new data receiver to process real or replayed data. This method only
   * works during setup before starting to log.
   */
  public static void addDataReceiver(LogDataReceiver dataReceiver) {
    Logger.addDataReceiver(dataReceiver);
  }

  /**
   * Registers a new dashboard input to be included in the periodic loop. This
   * function should not be called by the user.
   */
  public static void registerDashboardInput(LoggedDashboardInput dashboardInput) {
    Logger.registerDashboardInput(dashboardInput);
  }

  /**
   * Records a metadata value. This method only works during setup before starting
   * to log, then data will be recorded during the first cycle.
   * 
   * @param key   The name used to identify this metadata field.
   * @param value The value of the metadata field.
   */
  public static void recordMetadata(String key, String value) {
    Logger.recordMetadata(key, value);
  }

  /**
   * Causes the timestamp returned by "Timer.getFPGATimestamp()" and similar to
   * match the "real" time as reported by the FPGA instead of the logged time from
   * AdvantageKit.
   * 
   * <p>
   * Not recommended for most users as the behavior of the replayed
   * code will NOT match the real robot. Only use this method if your control
   * logic requires precise timestamps WITHIN a single cycle and you have no way
   * to move timestamp-critical operations to an IO interface. Also consider using
   * "getRealTimestamp()" for logic that doesn't need to match the replayed
   * version (like for analyzing performance).
   */
  public static void disableDeterministicTimestamps() {
    Logger.disableDeterministicTimestamps();
  }

  /**
   * Returns whether a replay source is currently being used.
   */
  public static boolean hasReplaySource() {
    return Logger.hasReplaySource();
  }

  /**
   * Starts running the logging system, including any data receivers or the replay
   * source.
   */
  public static void start(BreakerRobotStartConfig startConfig) {
    recordRobotMetadata(startConfig);
    Logger.start();
    logRobotStarted(startConfig);
  }

   public static void recordRobotMetadata(BreakerRobotStartConfig startConfig) {
    recordMetadata("Team", Integer.toString(startConfig.getTeamNum()));
    recordMetadata("Robot", startConfig.getRobotName());
    recordMetadata("BreakerLibVersion", BreakerLibVersion.Version);
    recordMetadata("RobotSoftware", startConfig.getRobotSoftwareVersion());
    recordMetadata("Authors", startConfig.getAuthorNames());
    recordMetadata("RobotControllerSerialNumber", RobotController.getSerialNumber());
    recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    recordMetadata("GitDate", BuildConstants.GIT_DATE);
    recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        recordMetadata("GitDirty", "Unknown");
        break;
    }
  }
   /** Startup message for robot. */
   public static void logRobotStarted(BreakerRobotStartConfig startConfig) {
    StringBuilder work = new StringBuilder(" | ---------------- ROBOT STARTED ---------------- |\n\n");
    work.append(" TEAM: " + startConfig.getTeamNum() + " - " + startConfig.getTeamName() + "\n");
    work.append(" ROBOT: " + startConfig.getRobotName() + " - " + startConfig.getRobotYear() + "\n");
    work.append(" BREAKERLIB: " + BreakerLibVersion.Version + " | " + "ROBOT SOFTWARE: "
        + startConfig.getRobotSoftwareVersion() + "\n");
    work.append(" AUTHORS: " + startConfig.getAuthorNames() + "\n\n");
    work.append(" | ---------------------------------------------- | \n\n\n");
    logEvent(work.toString());
  }
  /**
   * Logs robot mode change and plays enable tune. (automatically called by
   * BreakerRoboRIO)
   */
  public static void logRobotChangedMode(RobotOperatingMode newMode) {
    logEvent("| ---- ROBOT MODE CHANGED TO: " + newMode + " ---- |");
  }
  /** Logs given event. */
  public static void logEvent(String event) {
    String message = " EVENT: " + event;
    recordOutput("LoggedMessages/Events", message);
    System.out.println(message);
  }
  /**
   * Internal logging function for breakerlib classes to sepreate automated
   * breakerlib logging from user loging
   */
  public static void logBreakerLibEvent(String event) {
    String message = " BREAKERLIB INTERNAL EVENT: " + event;
    recordOutput("LoggedMessages/BreakerLibInternalEvents", message);
    System.out.println(message);
  }
  /**
   * Logs either exceptions thrown by code, or suer difigned errors either from
   * code or physical errors
   */
  public static void logError(String error) {
    String message = " ERROR: " + error;
    recordOutput("LoggedMessages/Errors", message);
    System.out.println(message);
  }
  public static void logError(Exception e) {
    logError(e.toString() + " : " + e.getStackTrace().toString());
  }
  /**
   * Logs robot superstructure (physical) events (i.e. intake activated, shooter
   * enabled)
   */
  public static void logSuperstructureEvent(String event) {
    String message = " ROBOT SUPERSTRUCTURE EVENT: " + event;
    recordOutput("LoggedMessages/SuperstructureEvents", message);
    System.out.println(message);
  }
  /** Write custom message to log. */
  public static void logMessage(String message) {
    recordOutput("LoggedMessages/General", message);
    System.out.println(message);
  }

  /**
   * Ends the logging system, including any data receivers or the replay source.
   */
  public static void end() {
    Logger.end();
  }

  /**
   * Returns the state of the receiver queue fault. This is tripped when the
   * receiver queue fills up, meaning that data is no longer being saved.
   */
  public static boolean getReceiverQueueFault() {
    return Logger.getReceiverQueueFault();
  }

  /**
   * Returns the current FPGA timestamp or replayed time based on the current log
   * entry (microseconds).
   */
  public static long getTimestamp() {
    return Logger.getTimestamp();
  }

  /**
   * Returns the true FPGA timestamp in microseconds, regardless of the timestamp
   * used for logging. Useful for analyzing performance. DO NOT USE this method
   * for any logic which might need to be replayed.
   */
  public static long getRealTimestamp() {
    return Logger.getRealTimestamp();
  }

  /**
   * Processes a set of inputs, logging them on the real robot or updating them in
   * the simulator. This should be called every loop cycle after updating the
   * inputs from the hardware (if applicable).
   * 
   * @param key    The name used to identify this set of inputs.
   * @param inputs The inputs to log or update.
   */
  public static void processInputs(String key, LoggableInputs inputs) {
    Logger.processInputs(key, inputs);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, byte[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, boolean value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, int value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, long value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, float value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, double value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, String value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <U extends Unit<U>> void recordOutput(String key, Measure<U> value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, boolean[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, int[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, long[] value) {
   Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, float[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, double[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, String[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a struct. Example usage:
   * {@code recordOutput("MyPose", Pose2d.struct, new Pose2d())}
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T> void recordOutput(String key, Struct<T> struct, T value) {
    Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct. Example usage:
   * {@code
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d(), new Pose2d());
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d[] {new Pose2d(), new
   * Pose2d()});
   * }
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <T> void recordOutput(String key, Struct<T> struct, T... value) {
    Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a protobuf. Protobuf should only be
   * used for objects that do not support struct serialization. Example usage:
   * {@code recordOutput("MyPose", Pose2d.proto, new Pose2d())}
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T, MessageType extends ProtoMessage<?>> void recordOutput(String key, Protobuf<T, MessageType> proto,
      T value) {
    Logger.recordOutput(key, proto, value);
  }

  // /**
  //  * Records a single output field for easy access when viewing the log. On the
  //  * simulator, use this method to record extra data based on the original inputs.
  //  * 
  //  * <p>
  //  * This method serializes a single object as a struct or protobuf automatically
  //  * by searching for a {@code struct} or {@code proto} field. Struct is preferred
  //  * if both methods are supported.
  //  * 
  //  * @param T     The type
  //  * @param key   The name of the field to record. It will be stored under
  //  *              "/RealOutputs" or "/ReplayOutputs"
  //  * @param value The value of the field.
  //  */
  // @SuppressWarnings("unchecked")
  // public static <T> void recordOutput(String key, T value) {
  //   Logger.recordOutput(key, value);
  // }

  // /**
  //  * Records a single output field for easy access when viewing the log. On the
  //  * simulator, use this method to record extra data based on the original inputs.
  //  * 
  //  * <p>
  //  * This method serializes an array of objects as a struct automatically
  //  * by searching for a {@code struct} field. Top-level protobuf arrays are not
  //  * supported.
  //  * 
  //  * @param T     The type
  //  * @param key   The name of the field to record. It will be stored under
  //  *              "/RealOutputs" or "/ReplayOutputs"
  //  * @param value The value of the field.
  //  */
  // @SuppressWarnings("unchecked")
  // public static <T> void recordOutput(String key, T... value) {
  //   Logger.recordOutput(key, value);
  // }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * The trajectory is logged as a series of poses.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, Trajectory value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, SwerveModuleState... value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * The current position of the Mechanism2d is logged once as a set of nested
   * fields. If the position is updated, this method must be called again.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void recordOutput(String key, Mechanism2d value) {
    Logger.recordOutput(key, value);
  }
  
}
