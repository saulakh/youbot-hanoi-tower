import matrix.Matrix;

public class YouBot {

    /**
     * Tb0 is the transformation matrix from the base frame of the arm to the mobile base / chassis frame
     * M0e is the end-effector configuration when the robot is in its zero configuration
     */

    final double DELTA_T = 0.01; // seconds
    final double MAX_SPEED = 15; // rad/s
    final double PI = 3.14159265;
    double[] errorIntegral = new double[6];

    // Chassis dimensions
    final double WHEEL_RADIUS = 0.0475; // meters
    final double FORWARD_BACKWARD_LENGTH = 0.235;
    final double SIDE_TO_SIDE_WIDTH = 0.15;

    // Initial youBot configuration (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    double[] initialConfig = new double[] {PI/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0};
    double[][] Tb0 = new double[][] {{1,0,0,0.1662},{0,1,0,0},{0,0,1,0.0026},{0,0,0,1}};
    double[][] M0e = new double[][] {{1,0,0,0.033},{0,1,0,0},{0,0,1,0.6546},{0,0,0,1}};
    double[][] BList = Matrix.transposeMatrix(new double[][] {{0,0,1,0,0.033,0},{0,-1,0,-0.5076,0,0},{0,-1,0,-0.3526,0,0},{0,-1,0,-0.2176,0,0},{0,0,1,0,0,0}});
    double[][] thetaList = Matrix.transposeArray(Matrix.rangeFromArray(initialConfig, 3, 8));

    // Starting configuration and controls
    double[] currentConfig = initialConfig;
    double[] currentControls = new double[9]; // 5 joint speeds, 4 wheel speeds (rad/s)

    // Initial and goal configurations of cube
    double[][] cubeInitial = new double[][] {{1,0,0,1},{0,1,0,0},{0,0,1,0.025},{0,0,0,1}};
    double[][] cubeGoal = new double[][] {{0,1,0,0},{-1,0,0,-1},{0,0,1,0.025},{0,0,0,1}};

    // Kp and Ki gains
    int KpGain = 5;
    int KiGain = 0;
    double[][] KpMatrix = Matrix.scalarMultiplication(Matrix.identityMatrix(6), KpGain);
    double[][] KiMatrix = Matrix.scalarMultiplication(Matrix.identityMatrix(6), KiGain);

    // Initial X, Xd, and XdNext
    double[][] X = new double[][] {{0.17,0,0.985,0.387},{0,1,0,0},{-0.985,0,0.17,0.57},{0,0,0,1}}; // Tse, current actual end-effector config
    double[][] Xd = new double[][] {{0,0,1,0.5},{0,1,0,0},{-1,0,0,0.5},{0,0,0,1}}; // current reference end-effector config
    double[][] XdNext = new double[][] {{0,0,1,0.6},{0,1,0,0},{-1,0,0,0.3},{0,0,0,1}}; // end-effector reference config at the next timestep

    public double cubicTimeScaling(int Tf, double t) {
        /*
        Computes s(t) for a cubic time scaling
        - Tf: Total time of the motion in seconds from rest to rest
        - t: The current time t satisfying 0 < t < Tf
        Output:
        - The path parameter s(t) corresponding to a third-order polynomial motion that begins and ends at zero velocity
         */
        return 3 * Math.pow(t / Tf, 2) - 2 * Math.pow(t / Tf, 3);
    }

    public double quinticTimeScaling(int Tf, double t) {
        /*
        Computes s(t) for a quintic time scaling
        - Tf: Total time of the motion in seconds from rest to rest
        - t: The current time t satisfying 0 < t < Tf
        Output:
        - The path parameter s(t) corresponding to a fifth-order polynomial motion that begins and ends at zero velocity and zero acceleration
         */
        return 10 * Math.pow(t / Tf, 3) - 15 * Math.pow(t / Tf, 4) + 6 * Math.pow(t / Tf, 5);
    }

    public double[][][] screwTrajectory(double[][] xStart, double[][] xEnd, int Tf, int N, int method) {
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
        double[][][] trajectory = new double[4][4][N];
        double s;

        for (int i=0; i < N; i++) {
            if (method == 3) {
                s = cubicTimeScaling(Tf, timeGap * i);
            } else {
                s = quinticTimeScaling(Tf, timeGap * i);
            }
            // trajectory[i] = TODO: Create MatrixExp6, MatrixLog6, dotProduct, and TransInv methods to finish this
        }
        return trajectory;
    }

    public double[][] adjointMatrix(double[][] T) {
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

    public double[][] matrixExp6(double[][] se3Matrix) {
        /*
        Computes the matrix exponential of an SE(3) representation of exponential coordinates
        - se3Matrix: A matrix in SE(3) representation
        Output:
        - Returns the matrix exponential of se3Matrix
         */
        // TODO: Finish MatrixExp6 method

        return se3Matrix;
    }

    public double[][] matrixLog6(double[][] se3Matrix) {
        /*
        Computes the matrix logarithm of a homogenous transformation matrix
        - se3Matrix: A matrix in SE(3) representation
        Output:
        - Returns the matrix logarithm of transformationMatrix
         */
        // TODO: Finish MatrixLog6 method
        return se3Matrix;
    }

    public double[][] transToRot(double[][] se3Matrix) {
        double[][] rotMatrix = new double[3][3];
        for (int i=0; i < rotMatrix.length; i++) {
            System.arraycopy(se3Matrix[i], 0, rotMatrix[i], 0, rotMatrix[0].length);
        }
        return rotMatrix;
    }

    public double[][] transToPos(double[][] se3Matrix) {
        double[][] posMatrix = new double[1][3];
        for (int i=0; i < posMatrix[0].length; i++) {
            posMatrix[0][i] = se3Matrix[i][3];
        }
        return posMatrix;
    }

    public double[][] transInv(double[][] matrix) {
        /*
        Inverts a homogenous transformation matrix
        - matrix: A homogenous transformation matrix
        Output:
        - Returns the inverse of the input matrix
        Uses the structure of transformation matrices to avoid taking a matrix inverse, for efficiency
         */
        // TODO: Finish transInv method
        double[][] transInv = new double[matrix.length][matrix[0].length];
        return transInv;
    }

    public double[][] vecToSo3(double[][] vector3x1) {
        /*
        Converts a 3-vector to so(3) representation
        - vector3x1: a 3-vector
        Output:
        - Returns the skew-symmetric representation of the input vector
         */
        double[][] so3 = new double[3][3];
        so3[0][1] = - vector3x1[0][2];
        so3[0][2] = vector3x1[0][1];
        so3[1][0] = vector3x1[0][2];
        so3[1][2] = - vector3x1[0][0];
        so3[2][0] = - vector3x1[0][1];
        so3[2][1] = vector3x1[0][0];
        return so3;
    }

}
