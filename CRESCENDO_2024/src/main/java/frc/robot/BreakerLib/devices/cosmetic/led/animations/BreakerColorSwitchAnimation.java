// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class BreakerColorSwitchAnimation extends Command implements BreakerAnimation {
    private int length, curColorIndex = 0;
    private double timePerColorSeconds;
    private boolean isStopped = true;
    private AddressableLEDBuffer buffer;
    private Color8Bit[] switchColors;
    private final Timer timer = new Timer();
    public BreakerColorSwitchAnimation(int length, double timePerColorSeconds, Color... colors) {
        this.length = length;
        this.timePerColorSeconds = timePerColorSeconds;
        switchColors = new Color8Bit[colors.length];
        for (int i = 0; i < colors.length; i++) {
            switchColors[i] = new Color8Bit(colors[i]);
        }
        buffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            buffer.setLED(i, switchColors[0]);
        }
    }

    @Override
    public void start() {
        isStopped = false;
        this.schedule();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public void stop() {
       isStopped = true;
    }

    @Override
    public boolean isActive() {
        return !isStopped;
    }

    @Override
    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void execute() {
        if (timer.get() >= timePerColorSeconds) {
            timer.reset();
            curColorIndex++;
            for (int i = 0; i < length; i++) {
                buffer.setLED(i, switchColors[curColorIndex]);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isStopped;
    }
    
}
