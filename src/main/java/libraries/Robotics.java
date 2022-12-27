package libraries;

import java.util.Arrays;

public class Robotics {

    /**
     * Rotates an SE(3) position about the x-axis by theta (radians)
     * @param initialSe3 SE(3) representation of initial position
     * @param theta Angle of rotation about the x-axis
     * @return The SE(3) position after rotating theta radians about the x-axis
     */
    public static double[][] rotateAboutX(double[][] initialSe3, double theta) {
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotX = {{1,0,0},{0, Math.cos(theta),-Math.sin(theta)},{0,Math.sin(theta),Math.cos(theta)}};
        Matrix.replaceRangeFromMatrix(rotX, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    /**
     * Rotates an SE(3) position about the y-axis by theta (radians)
     * @param initialSe3 SE(3) representation of initial position
     * @param theta Angle of rotation about the y-axis
     * @return The SE(3) position after rotating theta radians about the y-axis
     */
    public static double[][] rotateAboutY(double[][] initialSe3, double theta) {
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotY = {{Math.cos(theta),0,Math.sin(theta)},{0,1,0},{-Math.sin(theta),0,Math.cos(theta)}};
        Matrix.replaceRangeFromMatrix(rotY, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    /**
     * Rotates an SE(3) position about the z-axis by theta (radians)
     * @param initialSe3 SE(3) representation of initial position
     * @param theta Angle of rotation about the z-axis
     * @return The SE(3) position after rotating theta radians about the z-axis
     */
    public static double[][] rotateAboutZ(double[][] initialSe3, double theta) {
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotZ = {{Math.cos(theta),-Math.sin(theta),0},{Math.sin(theta),Math.cos(theta),0},{0,0,1}};
        Matrix.replaceRangeFromMatrix(rotZ, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    /**
     * Translates an SE(3) position in the x, y, and z directions, keeping its original orientation
     * @param initialSe3 SE(3) representation of initial position
     * @param x Change in position Δx across the x-axis
     * @param y Change in position Δy across the y-axis
     * @param z Change in position Δz across the z-axis
     * @return The SE(3) position after x, y, and z translations
     */
    public static double[][] translate(double[][] initialSe3, double x, double y, double z) {
        double[][] translate = Matrix.identityMatrix(4);
        translate[0][3] += x;
        translate[1][3] += y;
        translate[2][3] += z;
        return Matrix.matrixMultiplication(initialSe3, translate);
    }

    /**
     * Returns the SE(3) transformation matrix relative to the origin of the space frame {s}
     * @param thetaX Angle of rotation about the x-axis
     * @param thetaY Angle of rotation about the y-axis
     * @param thetaZ Angle of rotation about the z-axis
     * @param x Change in position Δx across the x-axis
     * @param y Change in position Δy across the y-axis
     * @param z Change in position Δz across the z-axis
     * @return SE(3) transformation after rotating and translating across the x, y, and z axes
     */
    public static double[][] transformationMatrix(double thetaX, double thetaY, double thetaZ, double x, double y, double z) {
        double[][] T = Matrix.identityMatrix(4);
        T = rotateAboutX(T, thetaX);
        T = rotateAboutY(T, thetaY);
        T = rotateAboutZ(T, thetaZ);
        return translate(T, x, y, z);
    }

    /**
     * Converts a homogenous transformation matrix into a rotation matrix
     * @param se3Matrix A homogenous transformation matrix
     * @return The corresponding rotation matrix
     */
    public static double[][] transToRot(double[][] se3Matrix) {
        double[][] rotMatrix = new double[3][3];
        for (int i=0; i < rotMatrix.length; i++) {
            System.arraycopy(se3Matrix[i], 0, rotMatrix[i], 0, rotMatrix[0].length);
        }
        return rotMatrix;
    }

    /**
     * Converts a homogenous transformation matrix into a position vector
     * @param se3Matrix A homogenous transformation matrix
     * @return The corresponding position vector
     */
    public static double[] transToPos(double[][] se3Matrix) {
        double[] posVector = new double[3];
        for (int i=0; i < posVector.length; i++) {
            posVector[i] = se3Matrix[i][3];
        }
        return posVector;
    }

    /**
     * Inverts a homogenous transformation matrix. Uses the structure of transformation matrices to avoid taking a matrix inverse, for efficiency.
     * @param matrix A homogenous transformation matrix
     * @return The inverse of the transformation matrix
     */
    public static double[][] transInv(double[][] matrix) {
        double[][] transInv = new double[matrix.length][matrix[0].length];
        double[][] rotTranspose = Matrix.transposeMatrix(transToRot(matrix));
        double[][] posTranspose = Matrix.transposeArray(transToPos(matrix));
        Matrix.replaceRangeFromMatrix(rotTranspose, transInv, 0, 0);
        double[][] rtPos = Matrix.matrixMultiplication(rotTranspose, posTranspose);
        assert rtPos != null;
        rtPos = Matrix.scalarMultiplication(rtPos, -1);
        transInv[0][3] = rtPos[0][0];
        transInv[1][3] = rtPos[1][0];
        transInv[2][3] = rtPos[2][0];
        transInv[3] = new double[] {0,0,0,1};
        return transInv;
    }

    /**
     * Converts a 3-vector of exponential coordinates for rotation into a unit rotation axis
     * @param vector A 3-vector of exponential coordinates for rotation
     * @return A unit rotation axis
     */
    public static double[] rotToAxis3(double[] vector) {
        double norm = rotToAng3(vector);
        return Matrix.scalarArrayMultiplication(vector, 1/norm);
    }

    /**
     * Calculates the corresponding angle from a 3-vector of exponential coordinates for rotation
     * @param vector A 3-vector of exponential coordinates for rotation
     * @return The corresponding rotation angle
     */
    public static double rotToAng3(double[] vector) {
        double norm = 0;
        for (double val : vector) {
            norm += Math.pow(val, 2);
        }
        return Math.pow(norm, 0.5);
    }

    /**
     * Converts a 3-vector to the so(3) representation
     * @param vector A 3-vector
     * @return Skew-symmetric so(3) representation of the input vector
     */
    public static double[][] vecToSo3(double[] vector) {
        double[][] so3 = new double[3][3];
        so3[0] = new double[] {0, -vector[2], vector[1]};
        so3[1] = new double[] {vector[2], 0, -vector[0]};
        so3[2] = new double[] {-vector[1], vector[0], 0};
        return so3;
    }

    /**
     * Converts the input so(3) representation to a 3-vector
     * @param so3Matrix A 3x3 skew-symmetric matrix
     * @return The 3-vector corresponding to the skew-symmetric representation
     */
    public static double[] so3ToVec(double[][] so3Matrix) {
        return new double[] {so3Matrix[2][1], so3Matrix[0][2], so3Matrix[1][0]};
    }

    /**
     * Converts a spatial velocity vector into a 4x4 matrix in se3
     * @param vector A 6-vector representing a spatial velocity
     * @return The 4x4 SE3 representation of the vector
     */
    public static double[][] vecToSE3(double[] vector) {
        double[][] vecToSe3 = new double[4][4];
        double[][] so3 = vecToSo3(Matrix.rangeFromArray(vector, 0, 3));
        Matrix.replaceRangeFromMatrix(so3, vecToSe3, 0, 0);
        double[][] vecLast3 = Matrix.transposeArray(Matrix.rangeFromArray(vector, 3, 6));
        Matrix.replaceRangeFromMatrix(vecLast3, vecToSe3, 0, 3);
        return vecToSe3;
    }

    /**
     * Converts se(3) matrix into a spatial velocity vector
     * @param se3Matrix A 4x4 matrix in se(3)
     * @return The spatial velocity 6-vector corresponding to the se(3) matrix
     */
    public static double[] se3ToVec(double[][] se3Matrix) {
        return new double[] {se3Matrix[2][1], se3Matrix[0][2], se3Matrix[1][0], se3Matrix[0][3], se3Matrix[1][3], se3Matrix[2][3]};
    }

    /**
     * Computes the adjoint representation of a homogenous transformation matrix
     * @param T A homogenous transformation matrix
     * @return 6x6 adjoint representation [AdT] of T
     */
    public static double[][] adjointMatrix(double[][] T) {
        double[][] adjoint = new double[6][6];
        double[][] rot = transToRot(T);
        double[][] skewPos = vecToSo3(transToPos(T));
        double[][] pR = Matrix.matrixMultiplication(skewPos, rot);
        assert pR != null;
        Matrix.replaceRangeFromMatrix(rot, adjoint, 0, 0);
        Matrix.replaceRangeFromMatrix(rot, adjoint, 3, 3);
        Matrix.replaceRangeFromMatrix(pR, adjoint, 3, 0);
        return adjoint;
    }

    /**
     * Computes the matrix exponential of a matrix in so(3)
     * @param matrix A 3x3 skew-symmetric matrix
     * @return Matrix exponential of the so(3) matrix
     */
    public static double[][] matrixExp3(double[][] matrix) {
        double[][] matrixExp3;
        double[][] identity = Matrix.identityMatrix(3);
        double theta = rotToAng3(so3ToVec(matrix));
        if (Math.abs(theta) < 0.000001) {
            return identity;
        }
        double[][] omgMatrix = Matrix.scalarMultiplication(matrix, 1/theta);
        double[][] omgMatrixSquared = Matrix.matrixMultiplication(omgMatrix, omgMatrix);
        matrixExp3 = Matrix.matrixAddition(identity, Matrix.scalarMultiplication(omgMatrix, Math.sin(theta)));
        assert matrixExp3 != null;
        assert omgMatrixSquared != null;
        return Matrix.matrixAddition(matrixExp3, Matrix.scalarMultiplication(omgMatrixSquared, (1 - Math.cos(theta))));
    }

    /**
     * Computes the matrix logarithm of a rotation matrix
     * @param rotMatrix A 3x3 rotation matrix
     * @return Matrix logarithm of the rotation matrix
     */
    public static double[][] matrixLog3(double[][] rotMatrix) {
        double acosInput = (rotMatrix[0][0] + rotMatrix[1][1] + rotMatrix[2][2] - 1) / 2.0;
        double[] omega;
        if (acosInput >= 1) {
            return new double[3][3];
        } else if (acosInput <= -1) {
            if ((Math.abs(1 + rotMatrix[2][2])) >= 0.000001) {
                omega = Matrix.scalarArrayMultiplication(new double[] {rotMatrix[0][2], rotMatrix[1][2], 1 + rotMatrix[2][2]}, 1.0 / Math.pow(2 * (1 + rotMatrix[2][2]), 0.5));
            } else if ((Math.abs(1 + rotMatrix[1][1])) >= 0.000001) {
                omega = Matrix.scalarArrayMultiplication(new double[] {rotMatrix[0][1], 1 + rotMatrix[1][1], rotMatrix[2][1]}, 1.0 / Math.pow(2 * (1 + rotMatrix[1][1]), 0.5));
            } else {
                omega = Matrix.scalarArrayMultiplication(new double[] {1 + rotMatrix[0][0], rotMatrix[1][0], 1+ rotMatrix[2][0]}, 1.0 / Math.pow(2 * (1 + rotMatrix[0][0]), 0.5));
            }
            return vecToSo3(Matrix.scalarArrayMultiplication(omega, Math.PI));
        }
        double theta = Math.acos(acosInput);
        double[][] negRotTranspose = Matrix.scalarMultiplication(Matrix.transposeMatrix(rotMatrix), -1);
        double[][] rotMinusRT = Matrix.matrixAddition(rotMatrix, negRotTranspose);
        double scalar = theta / (2.0 * Math.sin(theta));
        return Matrix.scalarMultiplication(rotMinusRT, scalar);
    }

    /**
     * Computes the matrix exponential of the se(3) representation of exponential coordinates
     * @param se3Matrix A matrix in se(3) representation
     * @return Matrix exponential of the se(3) matrix
     */
    public static double[][] matrixExp6(double[][] se3Matrix) {
        double[][] matrixExp6 = new double[4][4];
        double[][] rot = Matrix.rangeFromMatrix(se3Matrix, 0, 3, 0, 3);
        double[] pos = transToPos(se3Matrix);
        Matrix.replaceRangeFromMatrix(matrixExp3(rot), matrixExp6, 0, 0);

        double[][] identity = Matrix.identityMatrix(3);
        double theta = rotToAng3(so3ToVec(rot));
        if (Math.abs(theta) <= 0.000001) {
            Matrix.replaceRangeFromMatrix(Matrix.transposeArray(pos), matrixExp6, 0, 3);
            matrixExp6[3] = new double[] {0,0,0,1};
            return matrixExp6;
        }
        double[][] identityTheta = Matrix.scalarMultiplication(identity, theta);
        double[][] omgMatrix = Matrix.scalarMultiplication(rot, 1/theta);
        double[][] omgMatrixSquared = Matrix.matrixMultiplication(omgMatrix, omgMatrix);

        double[][] cosOmega = Matrix.scalarMultiplication(omgMatrix, (1 - Math.cos(theta)));
        assert omgMatrixSquared != null;
        double[][] sinOmegaSquared = Matrix.scalarMultiplication(omgMatrixSquared, (theta - Math.sin(theta)));
        double[][] v = Matrix.rangeFromMatrix(se3Matrix,0, 3, 3, 4);
        v = Matrix.scalarMultiplication(v, 1/theta);

        double[][] sum1 = Matrix.matrixAddition(identityTheta, cosOmega);
        double[][] sum = Matrix.matrixAddition(sum1, sinOmegaSquared);
        double[][] product = Matrix.matrixMultiplication(sum, v);
        Matrix.replaceRangeFromMatrix(product, matrixExp6, 0, 3);
        matrixExp6[3][3] = 1;
        return matrixExp6;
    }

    /**
     * Computes the matrix logarithm of a homogenous transformation matrix
     * @param se3Matrix A matrix in SE(3) representation
     * @return Matrix logarithm of the SE(3) matrix
     */
    public static double[][] matrixLog6(double[][] se3Matrix) {
        double[][] output = new double[4][4];
        double[][] rot = transToRot(se3Matrix);
        double[][] omgMatrix = matrixLog3(rot);
        double[][] pos = Matrix.rangeFromMatrix(se3Matrix, 0, 3, 3, 4);
        double theta = Math.acos((rot[0][0] + rot[1][1] + rot[2][2] - 1) / 2.0);

        if (Arrays.deepEquals(omgMatrix, new double[3][3])) {
            Matrix.replaceRangeFromMatrix(pos, output, 0, 3);
        }
        if (Math.abs(theta) <= 0.000001) {
            theta = 0.000001;
        }

        Matrix.replaceRangeFromMatrix(omgMatrix, output, 0, 0);
        double[][] identity = Matrix.identityMatrix(3);
        double[][] negHalfOmega = Matrix.scalarMultiplication(omgMatrix, -0.5);
        double[][] omegaSquared = Matrix.matrixMultiplication(omgMatrix, omgMatrix);
        double scalar = (1.0 / theta - 1.0 / Math.tan(theta / 2) / 2.0) / theta;
        double[][] scalarOmegaSquared = Matrix.scalarMultiplication(omegaSquared, scalar);

        double[][] m1 = Matrix.matrixAddition(identity, negHalfOmega);
        double[][] m2 = Matrix.matrixAddition(m1, scalarOmegaSquared);
        double[][] product = Matrix.matrixMultiplication(m2, pos);
        Matrix.replaceRangeFromMatrix(product, output, 0, 3);

        return output;
    }

    /**
     * Computes s(t) for a cubic time scaling
     * @param Tf Total time of the motion in seconds from rest to rest
     * @param t The current time t satisfying 0 < t < Tf
     * @return The path parameter s(t) corresponding to a third-order polynomial motion that begins and ends at zero velocity
     */
    public static double cubicTimeScaling(double Tf, double t) {
        return 3 * Math.pow(t / Tf, 2) - 2 * Math.pow(t / Tf, 3);
    }

    /**
     * Computes s(t) for a quintic time scaling
     * @param Tf Total time of the motion in seconds from rest to rest
     * @param t The current time t satisfying 0 < t < Tf
     * @return The path parameter s(t) corresponding to a fifth-order polynomial motion that begins and ends at zero velocity and zero acceleration
     */
    public static double quinticTimeScaling(double Tf, double t) {
        return 10 * Math.pow(t / Tf, 3) - 15 * Math.pow(t / Tf, 4) + 6 * Math.pow(t / Tf, 5);
    }

    /**
     * Computes a trajectory as a list of N SE(3) matrices corresponding to the screw motion about a space screw axis
     * @param xStart The initial end-effector configuration
     * @param xEnd The final end-effector configuration
     * @param Tf Total time of the motion in seconds from rest
     * @param N The number of points N > 1 (Start and stop) in the discrete representation of the trajectory
     * @param method The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling, and 5 indicates quintic (fifth-order polynomial) time scaling
     * @return The discretized trajectory as a list of N matrices in SE(3) separated in time by Tf/(N-1). The first in the list is xStart and the Nth is xEnd.
     */
    public static double[][][] screwTrajectory(double[][] xStart, double[][] xEnd, double Tf, int N, int method) {
        double timeGap = Tf / (N - 1.0);
        double[][][] trajectory = new double[N][4][4];
        double s;

        for (int i=0; i < N; i++) {
            if (method == 3) {
                s = cubicTimeScaling(Tf, timeGap * i);
            } else {
                s = quinticTimeScaling(Tf, timeGap * i);
            }
            double[][] xStartInvXEnd = Matrix.matrixMultiplication(transInv(xStart), xEnd);
            double[][] expInput = Matrix.scalarMultiplication(matrixLog6(xStartInvXEnd), s);
            trajectory[i] = Matrix.matrixMultiplication(xStart, matrixExp6(expInput));
        }
        return trajectory;
    }

    /**
     * Computes the body Jacobian for an open chain robot
     * @param BList The joint screw axes in the end-effector frame when the manipulator is at the home position,
     * in the format of a matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @return The body Jacobian corresponding to the inputs (6xn real numbers)
     */
    public static double[][] jacobianBody(double[][] BList, double[] thetaList) {
        double[][] jacobian = new double[BList.length][BList[0].length];
        for (int i=0; i < BList.length; i++) {
            jacobian[i][BList[0].length - 1] = BList[i][BList[0].length - 1];
        }
        double[][] T = Matrix.identityMatrix(4);

        for (int i=thetaList.length - 2; i > -1; i--) {
            double[] BListCol = Matrix.columnFromMatrix(BList, i);
            double[] BListNextCol = Matrix.columnFromMatrix(BList, i+1);
            double[] vecToSe3Input = Matrix.scalarArrayMultiplication(BListNextCol, -thetaList[i+1]);
            double[][] matrixExpInput = vecToSE3(vecToSe3Input);
            T = Matrix.matrixMultiplication(T, matrixExp6(matrixExpInput));
            double[][] adjTimesBListCol = Matrix.matrixMultiplication(adjointMatrix(T), Matrix.transposeArray(BListCol));
            Matrix.replaceRangeFromMatrix(adjTimesBListCol, jacobian, 0, i);
        }
        return jacobian;
    }

    /**
     * Computes forward kinematics in the body frame for an open chain robot
     * @param M The home configuration (position and orientation) of the end-effector
     * @param BList The joint screw axes in the end-effector frame when the manipulator is at the home position, as a matrix with axes as columns
     * @param thetaList A list of joint coordinates
     * @return Homogenous transformation matrix representing the end effector frame when the joints are at the specified coordinates (body frame)
     */
    public static double[][] fkInBody(double[][] M, double[][] BList, double[] thetaList) {
        double[][] T = M.clone();
        for (int i=0; i < thetaList.length; i++) {
            double[] BListCol = Matrix.columnFromMatrix(BList, i);
            double[] vecToSe3Input = Matrix.scalarArrayMultiplication(BListCol, thetaList[i]);
            double[][] matrixExp6Input = vecToSE3(vecToSe3Input);
            T = Matrix.matrixMultiplication(T, matrixExp6(matrixExp6Input));
        }
        return T;
    }

}
