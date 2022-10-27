package libraries;

import java.util.Arrays;

public class Robotics {

    public static double[][] rotateAboutX(double[][] initialSe3, double theta) {
        /*
        Rotates an SE(3) position about the x-axis by theta radians
         */
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotX = {{1,0,0},{0, Math.cos(theta),-Math.sin(theta)},{0,Math.sin(theta),Math.cos(theta)}};
        Matrix.replaceRangeFromMatrix(rotX, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    public static double[][] rotateAboutY(double[][] initialSe3, double theta) {
        /*
        Rotates an SE(3) position about the y-axis by theta radians
         */
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotY = {{Math.cos(theta),0,Math.sin(theta)},{0,1,0},{-Math.sin(theta),0,Math.cos(theta)}};
        Matrix.replaceRangeFromMatrix(rotY, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    public static double[][] rotateAboutZ(double[][] initialSe3, double theta) {
        /*
        Rotates an SE(3) position about the z-axis by theta radians
         */
        double[][] transMatrix = Matrix.identityMatrix(4);
        double[][] rotZ = {{Math.cos(theta),-Math.sin(theta),0},{Math.sin(theta),Math.cos(theta),0},{0,0,1}};
        Matrix.replaceRangeFromMatrix(rotZ, transMatrix, 0, 0);
        return Matrix.matrixMultiplication(initialSe3, transMatrix);
    }

    public static double[][] translate(double[][] initialSe3, double x, double y, double z) {
        /*
        Translates an SE(3) position in the x, y, and z directions, with its original orientation
         */
        initialSe3[0][3] += x;
        initialSe3[1][3] += y;
        initialSe3[2][3] += z;
        return initialSe3;
    }

    public static double[][] transToRot(double[][] se3Matrix) {
        /*
        Extracts the rotation matrix from a transformation matrix
         */
        double[][] rotMatrix = new double[3][3];
        for (int i=0; i < rotMatrix.length; i++) {
            System.arraycopy(se3Matrix[i], 0, rotMatrix[i], 0, rotMatrix[0].length);
        }
        return rotMatrix;
    }

    public static double[] transToPos(double[][] se3Matrix) {
        /*
        Extracts the position matrix from the transformation matrix
         */
        double[] posVector = new double[3];
        for (int i=0; i < posVector.length; i++) {
            posVector[i] = se3Matrix[i][3];
        }
        return posVector;
    }

    public static double[][] transInv(double[][] matrix) {
        /*
        Inverts a homogenous transformation matrix
        - matrix: A homogenous transformation matrix
        Output:
        - Returns the inverse of the input matrix
        Uses the structure of transformation matrices to avoid taking a matrix inverse, for efficiency
         */
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

    public static double[] rotToAxis3(double[] vector) {
        /*
        Calculates the corresponding axis from a 3-vector of exponential coordinates for rotation
         */
        double norm = rotToAng3(vector);
        return Matrix.scalarArrayMultiplication(vector, 1/norm);
    }

    public static double rotToAng3(double[] vector) {
        /*
        Calculates the corresponding angle from a 3-vector of exponential coordinates for rotation
         */
        double norm = 0;
        for (double val : vector) {
            norm += Math.pow(val, 2);
        }
        return Math.pow(norm, 0.5);
    }

    public static double[][] vecToSo3(double[] vector) {
        /*
        Converts a 3-vector to so(3) representation
        - vector: a 3-vector
        Output:
        - Returns the skew-symmetric representation of the input vector
         */
        double[][] so3 = new double[3][3];
        so3[0] = new double[] {0, -vector[2], vector[1]};
        so3[1] = new double[] {vector[2], 0, -vector[0]};
        so3[2] = new double[] {-vector[1], vector[0], 0};
        return so3;
    }

    public static double[] so3ToVec(double[][] so3Matrix) {
        /*
        Converts so(3) representation to a 3-vector
        - so3Matrix: A 3x3 skew-symmetric matrix
        Output:
        - Returns the 3-vector corresponding to the skew-symmetric representation
         */
        return new double[] {so3Matrix[2][1], so3Matrix[0][2], so3Matrix[1][0]};
    }

    public static double[][] vecToSE3(double[] vector) {
        /*
        Converts a spatial velocity vector into a 4x4 matrix in se3
        - vector: A 6-vector representing a spatial velocity
        Output:
        - Returns the 4x4 SE3 representation of the vector
         */
        double[][] vecToSe3 = new double[4][4];
        double[][] so3 = vecToSo3(Matrix.rangeFromArray(vector, 0, 3));
        Matrix.replaceRangeFromMatrix(so3, vecToSe3, 0, 0);
        double[][] vecLast3 = Matrix.transposeArray(Matrix.rangeFromArray(vector, 3, 6));
        Matrix.replaceRangeFromMatrix(vecLast3, vecToSe3, 0, 3);
        return vecToSe3;
    }

    public static double[] se3ToVec(double[][] se3Matrix) {
        /*
        Converts SE(3) matrix into a spatial velocity vector
        - se3Matrix: A 4x4 matrix in SE(3)
        Output:
        - Returns the spatial velocity 6-vector corresponding to the SE(3) matrix
         */
        return new double[] {se3Matrix[2][1], se3Matrix[0][2], se3Matrix[1][0], se3Matrix[0][3], se3Matrix[1][3], se3Matrix[2][3]};
    }

    public static double[][] adjointMatrix(double[][] T) {
        /*
        Computes the adjoint representation of a homogenous transformation matrix
        - T: A homogenous transformation matrix
        Output:
        - Returns the 6x6 adjoint representation [AdT] of T
         */
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

    public static double[][] matrixExp3(double[][] matrix) {
        /*
        Computes the matrix exponential of a matrix in so(3)
        - matrix: A 3x3 skew-symmetric matrix
        Output:
        - Returns the matrix exponential of the so(3) matrix
         */
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

    public static double[][] matrixExp6(double[][] se3Matrix) {
        /*
        Computes the matrix exponential of an SE(3) representation of exponential coordinates
        - se3Matrix: A matrix in SE(3) representation
        Output:
        - Returns the matrix exponential of the SE(3) matrix
         */
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

    public static double[][] matrixLog6(double[][] se3Matrix) {
        /*
        Computes the matrix logarithm of a homogenous transformation matrix
        - se3Matrix: A matrix in SE(3) representation
        Output:
        - Returns the matrix logarithm of the SE(3) matrix
         */
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

    public static double cubicTimeScaling(int Tf, double t) {
        /*
        Computes s(t) for a cubic time scaling
        - Tf: Total time of the motion in seconds from rest to rest
        - t: The current time t satisfying 0 < t < Tf
        Output:
        - The path parameter s(t) corresponding to a third-order polynomial motion that begins and ends at zero velocity
         */
        return 3 * Math.pow(t / Tf, 2) - 2 * Math.pow(t / Tf, 3);
    }

    public static double quinticTimeScaling(int Tf, double t) {
        /*
        Computes s(t) for a quintic time scaling
        - Tf: Total time of the motion in seconds from rest to rest
        - t: The current time t satisfying 0 < t < Tf
        Output:
        - The path parameter s(t) corresponding to a fifth-order polynomial motion that begins and ends at zero velocity and zero acceleration
         */
        return 10 * Math.pow(t / Tf, 3) - 15 * Math.pow(t / Tf, 4) + 6 * Math.pow(t / Tf, 5);
    }

    public static double[][][] screwTrajectory(double[][] xStart, double[][] xEnd, int Tf, int N, int method) {
        /*
        Computes a trajectory as a list of N SE(3) matrices corresponding to the screw motion about a space screw axis
        - xStart: The initial end-effector configuration
        - xEnd: The final end-effector configuration
        - Tf: Total time of the motion in seconds from rest
        - N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory
        - method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling, and 5 indicates quintic (fifth-order polynomial) time scaling
        Output:
        - Returns the discretized trajectory as a list of N matrices in SE(3) separated in time by Tf/(N-1). The first in the list is xStart and the Nth is xEnd.
         */
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

    public static double[][] jacobianBody(double[][] BList, double[] thetaList) {
        /*
        Computes the body Jacobian for an open chain robot
        - BList: The joint screw axes in the end-effector frame when the manipulator is at the home position,
        in the format of a matrix with axes as the columns
        - thetaList: A list of joint coordinates
        Output:
        - Returns the body Jacobian corresponding to the inputs (6xn real numbers)
         */
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

    public static double[][] fkInBody(double[][] M, double[][] BList, double[] thetaList) {
        /*
        Computes forward kinematics in the body frame for an open chain robot
        - M: The home configuration (position and orientation) of the end-effector
        - BList: The joint screw axes in the end-effector frame when the manipulator is at the home position, as a matrix with axes as columns
        Output:
        - Returns a homogenous transformation matrix representing the end effector frame when the joints are at the specified coordinates (body frame)
         */
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
