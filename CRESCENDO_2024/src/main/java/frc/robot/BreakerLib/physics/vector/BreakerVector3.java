// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.vector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * Represents a point with 3 axial vectors of ajustable magnitudes (one on each
 * X, Y, and Z axis)
 */
public class BreakerVector3 implements BreakerInterpolable<BreakerVector3> {

    private final double x, y, z, magnitude;
    private final Rotation3d vectorRotation;

    /** Creates a BreakerVector3 based on given x, y, and z forces.
     * 
     * @param x X-axis force relative to field.
     * @param y Y-axis force relative to field.
     * @param z Z-axis force relative to field.
    */
    public BreakerVector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        vectorRotation = new Rotation3d(0.0, Math.atan2(z, (Math.sqrt(x*x+y*y))),
                Math.atan2(y, x)); // need to check this math
    }

    public BreakerVector3() {
        x = 0;
        y = 0;
        z = 0;
        magnitude = 0;
        vectorRotation = new Rotation3d();
    }

    /**
     * Creates a BreakerVector3 from given magnitude and force rotation.
     * 
     * @param magnitude     Total force of the vector.
     * @param vectorRotation Rotation of forces.
     */
    public BreakerVector3(double magnitude, Rotation3d vectorRotation) {
        x = magnitude * (Math.cos(vectorRotation.getZ()) * Math.cos(vectorRotation.getY()));
        y = magnitude * (Math.sin(vectorRotation.getZ()) * Math.cos(vectorRotation.getY()));
        z = magnitude * (Math.sin(vectorRotation.getY()));
        this.magnitude = magnitude;
        this.vectorRotation = vectorRotation;
    }

    /** converts an instance of WPILib's translation3d class into a vector.
     *  This exists because of the Tranlation2d classes suppport of various vector opperations */
    public BreakerVector3(Translation3d translationToVectorize) {
        this(translationToVectorize.getX(), translationToVectorize.getY(), translationToVectorize.getY());
    }


    public static BreakerVector3 fromRowMatrix(Matrix<N1, N3> rowMatrix) {
        return new BreakerVector3(rowMatrix.get(0, 0), rowMatrix.get(0, 1), rowMatrix.get(0, 2));
    }

    public static BreakerVector3 fromColumnMatrix(Matrix<N3, N1> columnMatrix) {
        return new BreakerVector3(columnMatrix.get(0, 0), columnMatrix.get(1, 0), columnMatrix.get(2, 0));
    }

    /** 
     * @return double
     */
    public double getX() {
        return x;
    }
    
    /** 
     * @return double
     */
    public double getY() {
        return y;
    }
  
    /** 
     * @return double
     */
    public double getZ() {
        return z;
    }
    
    /** 
     * @return double
     */
    public double getMagnitude() {
        return magnitude;
    }
    
    /** 
     * @return Rotation3d
     */
    public Rotation3d getVectorRotation() {
        return vectorRotation;
    }

    public BreakerVector3 clampMagnitude(double low, double high) {
        double clampedMag = MathUtil.clamp(magnitude, low, high);
        return new BreakerVector3(clampedMag, vectorRotation);
    }

    public BreakerVector3 abs() {
        return new BreakerVector3(Math.abs(x), Math.abs(y), Math.abs(z));
    }

    /** 
     * @param rotation
     * @return BreakerVector3
     */
    public BreakerVector3 rotateBy(Rotation3d rotation) {
        return new BreakerVector3(magnitude, vectorRotation.plus(rotation));
    }

    /** 
     * @param outher
     * @return BreakerVector3
     */
    public BreakerVector3 minus(BreakerVector3 outher) {
        return new BreakerVector3(x - outher.x, y - outher.y, z - outher.z);
    }
    
    /** 
     * @param outher
     * @return BreakerVector3
     */
    public BreakerVector3 plus(BreakerVector3 outher) {
        return new BreakerVector3(x + outher.x, y + outher.y, z + outher.z);
    }
    
    /** 
     * @return BreakerVector3
     */
    public BreakerVector3 unaryMinus()  {
        return new BreakerVector3(-x, -y, -z);
    }
    
    /** 
     * @param scalar
     * @return BreakerVector3
     */
    public BreakerVector3 times(double scalar) {
        return new BreakerVector3(x * scalar, y * scalar, z * scalar);
    }
    
    /** 
     * @param scalar
     * @return BreakerVector3
     */
    public BreakerVector3 div(double scalar) {
        return new BreakerVector3(x / scalar,  y / scalar, y / scalar);
    }
    
    /** 
     * @return Translation3d
     */
    public Translation3d getAsTranslation() {
        return new Translation3d(x, y, z);
    }

    public BreakerVector2 toBreakerVector2() {
        return new BreakerVector2(x, y);
    }

    /** 
     * @return BreakerVector3
     */
    public BreakerVector3 getUnitVector() {
        return new BreakerVector3(1, vectorRotation);
    }

    public Matrix<N3, N1> getColumnMatrix() {
        Matrix<N3, N1> mtrx = new Matrix<N3,N1>(Nat.N3(), Nat.N1());
        mtrx.set(0, 0, x);
        mtrx.set(1, 0, y);
        mtrx.set(2, 0, z);
        return mtrx;
    }

    public Matrix<N1, N3> getRowMatrix() {
        Matrix<N1, N3> mtrx = new Matrix<N1,N3>(Nat.N1(), Nat.N3());
        mtrx.set(0, 0, x);
        mtrx.set(0, 1, y);
        mtrx.set(0, 2, z);
        return mtrx;
    }

    /** 
     * @param endValue
     * @param t
     * @return BreakerVector3
     */
    @Override
    public BreakerVector3 interpolate(BreakerVector3 endValue, double t) {
        double interX = MathUtil.interpolate(x, endValue.getX(), t);
        double interY = MathUtil.interpolate(y, endValue.getY(), t);
        double interZ = MathUtil.interpolate(z, endValue.getZ(), t);
        return new BreakerVector3(interX, interY, interZ);
    }
    
    /** [0] = X, [1] = Y, [2] = Z */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { x, y, z };
    }

    
    /** 
     * @param interpolatableData
     * @return BreakerVector3
     */
    @Override
    public BreakerVector3 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector3(interpolatableData[0], interpolatableData[1], interpolatableData[2]);
    }

    
    /** 
     * @return String
     */
    @Override
    public String toString() {
        return String.format("BreakerVector3(Magnatude: %.2f, X: %.2f, Y: %.2f, Z: %.2f, Angle: %s)", magnitude, x, y, z, vectorRotation.toString());
    }
}
