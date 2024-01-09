// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import java.awt.Shape;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

/** A class that represents the geometric 2d layout of an FRC play field */
public class BreakerFieldGeometry2d {
    private Shape[] componentShapes;

    public BreakerFieldGeometry2d(Shape... componentShapes) {
        this.componentShapes = componentShapes;
    }

    public Shape[] getComponentShapes() {
        return componentShapes.clone();
    }

    public boolean intersects(Rectangle2D rectangle) {
        for (Shape s: componentShapes) {
            if (s.intersects(rectangle)) {
                return true;
            }
        }
        return false;
    }

    public boolean intersects(Shape shape) {
        for (Shape s: componentShapes) {
            if (s.intersects(shape.getBounds2D())) {
                return true;
            }
        }
        return false;
    }

    public boolean contains(Rectangle2D rectangle) {
        for (Shape s: componentShapes) {
            if (s.contains(rectangle)) {
                return true;
            }
        }
        return false;
    }

    public boolean contains(Point2D point) {
        for (Shape s: componentShapes) {
            if (s.contains(point)) {
                return true;
            }
        }
        return false;
    }

    public boolean contains(Shape shape) {
        for (Shape s: componentShapes) {
            if (s.contains(shape.getBounds2D())) {
                return true;
            }
        }
        return false;
    }

    public boolean robotHitboxIntersects(BreakerRobotGeometry2d robotGeometry, Pose2d robotPose) {
       return intersects(robotGeometry.getTranslatedHitbox(robotPose));
    }

    public boolean trajectoryIntersects(Trajectory trajectoryCurve, BreakerRobotGeometry2d robotGeometry, double dt) {
        double time = trajectoryCurve.getTotalTimeSeconds();
        for (double t = 0; t <= 1.0; t += dt) {
            State sample = trajectoryCurve.sample(time * t);
            if (robotHitboxIntersects(robotGeometry, sample.poseMeters)) {
                return true;
            }
        }
        return false;
    }

    

}
