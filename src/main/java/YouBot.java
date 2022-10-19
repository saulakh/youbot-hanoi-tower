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

    // H(0) matrix: Chassis kinematic model
    double[][] H = Matrix.scalarMultiplication(new double[][] {{-FORWARD_BACKWARD_LENGTH - SIDE_TO_SIDE_WIDTH, 1, -1},
            {FORWARD_BACKWARD_LENGTH + SIDE_TO_SIDE_WIDTH, 1, 1},
            {FORWARD_BACKWARD_LENGTH + SIDE_TO_SIDE_WIDTH, 1, -1},
            {-FORWARD_BACKWARD_LENGTH - SIDE_TO_SIDE_WIDTH, 1, 1}}, 1 / WHEEL_RADIUS);
    double[][] F = Matrix.pseudoInverse(H);

    // Initial youBot configuration (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    double[] initialConfig = new double[] {PI/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0};
    double[][] Tb0 = new double[][] {{1,0,0,0.1662},{0,1,0,0},{0,0,1,0.0026},{0,0,0,1}};
    double[][] M0e = new double[][] {{1,0,0,0.033},{0,1,0,0},{0,0,1,0.6546},{0,0,0,1}};
    double[][] BList = Matrix.transposeMatrix(new double[][] {{0,0,1,0,0.033,0},{0,-1,0,-0.5076,0,0},{0,-1,0,-0.3526,0,0},{0,-1,0,-0.2176,0,0},{0,0,1,0,0,0}});
    double[] thetaList = Matrix.rangeFromArray(initialConfig, 3, 8);

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

}
