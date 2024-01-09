package frc.robot.BreakerLib.util.logging.advantagekit;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A set of values which can be logged and replayed (for example, the hardware
 * inputs for a subsystem). Data is stored in LogTable objects.
 */
public interface BreakerLoggable extends LoggableInputs {
  public void toLog(LogTable table);

  @Override
  default void fromLog(LogTable table) {}

}
