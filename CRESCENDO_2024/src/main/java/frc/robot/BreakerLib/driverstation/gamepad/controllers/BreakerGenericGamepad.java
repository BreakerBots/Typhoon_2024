package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerAnalogTrigger;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerDPad;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerFaceButtons;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadThumbstick;

/** BreakerLib gamepad interface. */
public class BreakerGenericGamepad {
    protected GenericHID hid;
    protected BreakerFaceButtons faceButtons;
    protected BreakerGamepadThumbstick leftJoystick, rightJoystick;
    protected BreakerAnalogTrigger leftTrigger, rightTrigger;
    protected JoystickButton leftBumper, rightBumper;
    protected BreakerDPad dPad;

    /** Complete constructor for all BreakerGenericGamepads. Not intended for end users. */
    protected BreakerGenericGamepad(
            GenericHID hid,
            int upFaceButtonPort,
            int leftFaceButtonPort,
            int rightFaceButtonPort,
            int downFaceButtonPort,
            int leftThumbstickButtonPort,
            int leftThumbstickAxisPortX,
            boolean invertLeftThumbstickAxisPortX,
            int leftThumbstickAxisPortY,
            boolean invertLeftThumbstickAxisPortY,
            int rightThumbstickButtonPort,
            int rightThumbstickAxisPortX,
            boolean invertRightThumbstickAxisPortX,
            int rightThumbstickAxisPortY,
            boolean invertRightThumbstickAxisPortY,
            int leftTriggerAxisPort,
            int rightTriggerAxisPort,
            int leftBumperPort,
            int rightBumperPort) {

        this.hid = hid;
        faceButtons = new BreakerFaceButtons(hid, upFaceButtonPort, leftFaceButtonPort,
                rightFaceButtonPort, downFaceButtonPort);
        leftJoystick = new BreakerGamepadThumbstick(hid, leftThumbstickButtonPort, leftThumbstickAxisPortX,
                invertLeftThumbstickAxisPortX, leftThumbstickAxisPortY, invertLeftThumbstickAxisPortY);
        rightJoystick = new BreakerGamepadThumbstick(hid, rightThumbstickButtonPort, rightThumbstickAxisPortX,
                invertRightThumbstickAxisPortX, rightThumbstickAxisPortY, invertRightThumbstickAxisPortY);
        leftTrigger = new BreakerAnalogTrigger(hid, leftTriggerAxisPort);
        rightTrigger = new BreakerAnalogTrigger(hid, rightTriggerAxisPort);
        leftBumper = new JoystickButton(hid, leftBumperPort);
        rightBumper = new JoystickButton(hid, rightBumperPort);
        dPad = new BreakerDPad(hid);
    }

    /**
     * Configures all analog deadbands.
     * 
     * @param deadbandConfig List of deadbands.
     */
    public void configDeadbands(BreakerGamepadAnalogDeadbandConfig deadbandConfig) {
        leftJoystick.setDeadband(deadbandConfig.getLeftStick());
        rightJoystick.setDeadband(deadbandConfig.getRightStick());
        leftTrigger.setDeadband(deadbandConfig.getLeftTriggerAxis());
        rightTrigger.setDeadband(deadbandConfig.getRightTriggerAxis());
    }

    /**
     * Sets the controller to rumble.
     * 
     * @param rumbleType Fine, coarse, or mixed.
     * @param rumblePercent Percent value to rumble.
     */
    public void setRumble(BreakerControllerRumbleType rumbleType, double rumblePercent) {
        switch (rumbleType) {
            case LEFT:
                hid.setRumble(RumbleType.kLeftRumble, rumblePercent);
                break;
            case RIGHT:
                hid.setRumble(RumbleType.kRightRumble, rumblePercent);
                break;
            case MIXED:
            default:
                setMixedRumble(rumblePercent, rumblePercent);
                break;
        }
    }

    /**
     * Set asymmetric rumbling percents for mixed rumble mode.
     * 
     * @param leftRumble Coarse rumble %.
     * @param rightRumble Fine rumble %.
     */
    public void setMixedRumble(double leftRumble, double rightRumble) {
        hid.setRumble(RumbleType.kLeftRumble, leftRumble);
        hid.setRumble(RumbleType.kRightRumble, leftRumble);
    }

    /** Sets rumble to 0. */
    public void clearRumble() {
        setMixedRumble(0, 0);
    }

    /** @return Controller's {@link BreakerFaceButton} object. */
    public BreakerFaceButtons getFaceButtons() {
        return faceButtons;
    }

    /** @return Controller's {@link BreakerGamepadThumbstick} object for L stick. */
    public BreakerGamepadThumbstick getLeftThumbstick() {
        return leftJoystick;
    }

    /** @return Controller's {@link BreakerGamepadThumbstick} object for R stick. */
    public BreakerGamepadThumbstick getRightThumbstick() {
        return rightJoystick;
    }

    /** @return Controller's {@link BreakerAnalogTrigger} object for left trigger. */
    public BreakerAnalogTrigger getLeftTrigger() {
        return leftTrigger;
    }

    /** @return Controller's {@link BreakerAnalogTrigger} object for right trigger. */
    public BreakerAnalogTrigger getRightTrigger() {
        return rightTrigger;
    }

    /** @return Controller's {@link BreakerDPad} object. */
    public BreakerDPad getDPad() {
        return dPad;
    }

    /** @return Controller's {@link JoystickButton} object for left bumper. */
    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    /** @return Controller's {@link JoystickButton} object for right bumper. */
    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    /** @return Base {@link GenericHID}. */
    public GenericHID getBaseHID() {
        return hid;
    }

}