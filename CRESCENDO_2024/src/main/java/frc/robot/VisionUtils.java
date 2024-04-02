// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionUtils {
    public static SimpleMatrix createTransformMatrix(Transform3d transform) {
        SimpleMatrix rotationMatrix = quaternionToRotationMatrix(transform.getRotation().getQuaternion());
        SimpleMatrix translationMatrix = new SimpleMatrix(3, 1);
        translationMatrix.set(0, 0, transform.getX());
        translationMatrix.set(1, 0, transform.getY());
        translationMatrix.set(2, 0, transform.getZ());

        SimpleMatrix transformMatrix = new SimpleMatrix(4, 4);
        transformMatrix.insertIntoThis(0, 0, rotationMatrix);
        transformMatrix.insertIntoThis(0, 3, translationMatrix);
        transformMatrix.set(3, 3, 1);

        return transformMatrix;
    }

    public static SimpleMatrix quaternionToRotationMatrix(Quaternion quat) {
        double sqw = quat.getW() * quat.getW();
        double sqx = quat.getX() * quat.getX();
        double sqy = quat.getY() * quat.getY();
        double sqz = quat.getZ() * quat.getZ();

        double invs = 1 / (sqx + sqy + sqz + sqw);

        double m00 = (sqx - sqy - sqz + sqw) * invs;
        double m11 = (-sqx + sqy - sqz + sqw) * invs;
        double m22 = (-sqx - sqy + sqz + sqw) * invs;

        double tmp1 = quat.getX() * quat.getY();
        double tmp2 = quat.getZ() * quat.getW();
        double m10 = 2.0 * (tmp1 + tmp2) * invs;
        double m01 = 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = quat.getX() * quat.getY();
        tmp2 = quat.getY() * quat.getW();
        double m20 = 2.0 * (tmp1 - tmp2) * invs;
        double m02 = 2.0 * (tmp1 + tmp2) * invs;

        tmp1 = quat.getY() * quat.getZ();
        tmp2 = quat.getX() * quat.getW();
        double m21 = 2.0 * (tmp1 + tmp2) * invs;
        double m12 = 2.0 * (tmp1 - tmp2) * invs;

        SimpleMatrix rotationMatrix = new SimpleMatrix(3, 3);
        rotationMatrix.set(0, 0, m00);
        rotationMatrix.set(1, 1, m11);
        rotationMatrix.set(2, 2, m22);
        rotationMatrix.set(1, 0, m10);
        rotationMatrix.set(0, 1, m01);
        rotationMatrix.set(2, 0, m20);
        rotationMatrix.set(0, 2, m02);
        rotationMatrix.set(2, 1, m21);
        rotationMatrix.set(1, 2, m12);

        return rotationMatrix;
    }

    public static Translation3d triangulateRobotToPoint(Transform3d cameraToRobotTransform1, Transform3d cameraToRobotTransform2, double yawCam1, double pitchCam1, double yawCam2, double pitchCam2) {
        SimpleMatrix cameraToRobotTransformMatrix1 = createTransformMatrix(cameraToRobotTransform1);
        SimpleMatrix cameraToRobotTransformMatrix2 = createTransformMatrix(cameraToRobotTransform2);
        SimpleMatrix robotToPointMatrix = triangulatePointMatrix(cameraToRobotTransformMatrix1, cameraToRobotTransformMatrix2, yawCam1, pitchCam1, yawCam2, pitchCam2);
        double x = robotToPointMatrix.get(0,0);
        double y = robotToPointMatrix.get(1,0);
        double z = robotToPointMatrix.get(2,0);
        return new Translation3d(x, y, z);
    }

    private static SimpleMatrix triangulatePointMatrix(SimpleMatrix camera1Transform, SimpleMatrix camera2Transform,
                                                double yawCamera1, double pitchCamera1,
                                                double yawCamera2, double pitchCamera2) {
        // Calculate direction vectors of the rays from each camera to the point
        SimpleMatrix directionVectorCamera1 = calculateDirectionVector(yawCamera1, pitchCamera1);
        SimpleMatrix directionVectorCamera2 = calculateDirectionVector(yawCamera2, pitchCamera2);

        // Convert direction vectors to unit vectors
        directionVectorCamera1 = directionVectorCamera1.divide(directionVectorCamera1.normF());
        directionVectorCamera2 = directionVectorCamera2.divide(directionVectorCamera2.normF());

        // Triangulate the point using the direction vectors and camera positions
        SimpleMatrix triangulatedPoint = triangulate(directionVectorCamera1, directionVectorCamera2,
                                                     camera1Transform, camera2Transform);

        return triangulatedPoint;
    }

    private static SimpleMatrix calculateDirectionVector(double yaw, double pitch) {
        double x = Math.sin(yaw) * Math.cos(pitch);
        double y = Math.sin(pitch);
        double z = Math.cos(yaw) * Math.cos(pitch);

        return new SimpleMatrix(new double[][]{{x}, {y}, {z}});
    }

    private static SimpleMatrix triangulate(SimpleMatrix directionVector1, SimpleMatrix directionVector2,
                                            SimpleMatrix camera1Transform, SimpleMatrix camera2Transform) {
        // Calculate relative position of the second camera with respect to the first camera
        SimpleMatrix relativeTransform = camera1Transform.invert().mult(camera2Transform);

        // Calculate the direction vector from the first camera to the point in the first camera's coordinate system
        SimpleMatrix directionVector1InCamera1Frame = relativeTransform.mult(directionVector2);

        // Calculate the scale factor to intersect the two rays
        double scaleFactor = directionVector1.transpose().mult(directionVector1InCamera1Frame).get(0);

        // Calculate the triangulated point in the first camera's coordinate system
        SimpleMatrix triangulatedPointInCamera1Frame = directionVector1.scale(scaleFactor);

        return triangulatedPointInCamera1Frame;
    }
}
