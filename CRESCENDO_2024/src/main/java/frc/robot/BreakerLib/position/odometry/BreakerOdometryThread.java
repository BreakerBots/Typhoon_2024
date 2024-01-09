// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ejml.equation.Function;
import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.NotifierCommand;

/** Provides notifier for high frequency odometry. WARNING: Mutex Locks must be user implemnted on all calls, this class is not inherently thread safe */
public abstract class BreakerOdometryThread extends Thread implements BreakerGenericOdometer {
    protected ReentrantLock odometryLock;
    private int lastThreadPriority;
    private int threadPriorityToSet;
    private double currentTime;
    private double lastTime;
    private MedianFilter peakRemover;
    private LinearFilter lowPass;
    private double averagePeriod;
    protected BreakerOdometryThread(int threadPriority) {
        setDaemon(true);
        peakRemover = new MedianFilter(3);
        lowPass = LinearFilter.movingAverage(50);
        lastThreadPriority = threadPriority;
        threadPriorityToSet = threadPriority;
    }

    public void setThreadPriority(int priority) {
        threadPriorityToSet = priority;
    }

    protected abstract void init();
    /** NOTE: This function is not mutex protected */
    protected abstract void periodControl();
    protected abstract void update();
    public double getPeriod() {
        return averagePeriod;
    }

    private void calculatePeriod() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        averagePeriod = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));
    }


    @Override
    public void run() {
        Threads.setCurrentThreadPriority(true, threadPriorityToSet);
        odometryLock.lock();
        init();
        odometryLock.unlock();
        currentTime = Timer.getFPGATimestamp();
        while(true) {
            periodControl();
            odometryLock.lock();
            calculatePeriod();
            update();
            odometryLock.unlock();
            if (threadPriorityToSet != lastThreadPriority) {
                Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                lastThreadPriority = threadPriorityToSet;
            }
        }
    }
}
