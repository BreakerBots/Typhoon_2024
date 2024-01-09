// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class BreakerRainbowAnimation extends Command implements BreakerAnimation {
    
    private int length;
    private double speed, brightness;
    private boolean isStoped = true;
    private AddressableLEDBuffer buff;
    private Color8Bit[] baseColorGrade;
    private double accumInc = 0;
    public BreakerRainbowAnimation(int length, double speed, double brightness) {
        this.length = length;
        this.brightness = brightness;
        this.speed = speed;
        buff = new AddressableLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            baseColorGrade[i] = new Color8Bit(Color.fromHSV((int) (180.0 * (double) (i/(length - 1))), 255,(int) (brightness * 255)));
        }
        for (int i = 0; i < length; i++) {
            buff.setLED(i, baseColorGrade[i]);
        }
    }

    @Override
    public void start() {
        isStoped = false;
        this.schedule();
    }

    @Override
    public void stop() {
       isStoped = true;
    }

    @Override
    public boolean isActive() {
        return !isStoped;
    }

    @Override
    public AddressableLEDBuffer getBuffer() {
        return buff;
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void execute() {
        accumInc += speed;
        for (int i = 0; i < length; i++) {
            int incIndex =(int) MathUtil.inputModulus(i + accumInc, 0, length - 1);
            buff.setLED(i, baseColorGrade[incIndex]);
        }
    }

    @Override
    public boolean isFinished() {
        return isStoped;
    }
    
}
