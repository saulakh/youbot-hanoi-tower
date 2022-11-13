package path;

import libraries.Matrix;
import libraries.Robotics;
import model.YouBot;

import java.util.ArrayList;
import java.util.List;

public class FeedbackControl {

    private final YouBot robot;

    public FeedbackControl(YouBot youBot) {
        this.robot = youBot;
    }

    /**
     * Returns an array of 5 joint angles from the current robot configuration
     * @return thetaList : array of 5 joint angles for robot arm configuration
     */
    private double[] getThetaList() {
        return Matrix.rangeFromArray(robot.currentConfig, 3, 8);
    }

    /**
     * Returns the transformation matrix from base to end-effector using forward kinematics
     * @return T0e : base to end-effector transformation matrix (SE3)
     */
    private double[][] getT0e() {
        return Robotics.fkInBody(robot.M0e, robot.BList, getThetaList());
    }

    /**
     * Returns the F6 matrix, using the (3xm) F matrix (pseudo-inverse of the chassis kinematic model H(θ))
     * @return F6Matrix : 6 x m matrix with two rows of m zeros stacked above F and one row below it
     */
    private double[][] getF6Matrix() {
        int m = robot.F[0].length;
        double[][] F6Matrix = new double[6][m];
        Matrix.replaceRangeFromMatrix(robot.F, F6Matrix, 2, 0);
        return F6Matrix;
    }

    /**
     * Returns the body Jacobian matrix J_arm(θ), which represents the relationship between
     * the joint rates and the end-effector's twist expressed in the fixed space-frame coordinates
     * @return armJacobian : the Jacobian in the end-effector (or body-) frame coordinates, expressing the
     * contribution of joint velocities to the end-effector's velocity
     */
    private double[][] getArmJacobian() {
        return Robotics.jacobianBody(robot.BList, getThetaList());
    }

    /**
     * Returns the base Jacobian matrix J_base(θ), which represents the relationship between the wheel speeds and the
     * end-effector's twist expressed in the fixed space-frame coordinates
     * @return baseJacobian : the base Jacobian, expressing the contribution of the wheeled velocities u to
     * the end-effector's velocity
     */
    private double[][] getBaseJacobian() {
        double[][] Te0 = Matrix.inverseMatrix(getT0e());
        double[][] Te0Adjoint = Robotics.adjointMatrix(Te0);
        double[][] F6Matrix = getF6Matrix();

        return Matrix.matrixMultiplication(Te0Adjoint, F6Matrix);
    }

    /**
     * Concatenates the arm Jacobian and base Jacobian to get the full Jacobian Je(θ)
     * @return fullJacobian : complete Jacobian matrix representing the wheel and joint velocities, used for
     * kinematic control of the end-effector frame in the fixed space-frame coordinates
     */
    public double[][] getFullJacobian() {
        double[][] armJacobian = getArmJacobian();
        double[][] baseJacobian = getBaseJacobian();
        double[][] fullJacobian = new double[6][baseJacobian[0].length + armJacobian[0].length];

        Matrix.replaceRangeFromMatrix(armJacobian, fullJacobian, 0, 0);
        Matrix.replaceRangeFromMatrix(baseJacobian, fullJacobian, 0, armJacobian[0].length);
        return fullJacobian;
    }

    /**
     * Returns a list of joint angles that exceed joint limits
     * @param currentConfig robot configuration (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4)
     * @return constrainJoints : list of joint numbers (1 - 5) that need to be constrained
     */
    private List<Integer> testJointLimits(double[] currentConfig) {
        List<Integer> constrainJoints = new ArrayList<>();
        double theta3 = currentConfig[5];
        double theta4 = currentConfig[6];
        if (theta3 < -2 || theta3 > -0.2) {
            constrainJoints.add(3);
        }
        if (theta4 < -2 || theta4 > 2) {
            constrainJoints.add(4);
        }
        return constrainJoints;
    }

    /**
     * Returns the error twist xErr = log (xInv * Xd)
     * @return xErr : error twist between the current and reference positions
     */
    private double[] getErrorTwist() {
        double[][] matrixLog6Input = Matrix.matrixMultiplication(Matrix.inverseMatrix(robot.X), robot.Xd);
        double[][] se3ToVecInput = Robotics.matrixLog6(matrixLog6Input);
        return Robotics.se3ToVec(se3ToVecInput);
    }

    /**
     * Calculates the feedforward reference twist that takes the end-effector position from Xd to XdNext in time Δt,
     * using Vd = (1 / Δt) log (XdInv * XdNext)
     * @return VdAdjoint : Adjoint representation of the feedforward reference twist Vd
     */
    private double[] getFeedForwardTwist() {

        double[][] matrixLog6Input = Matrix.matrixMultiplication(Matrix.inverseMatrix(robot.Xd), robot.XdNext);
        double[][] se3ToVecInput = Matrix.scalarMultiplication(matrixLog6Input, 1/robot.DELTA_T);
        double[] Vd = Robotics.se3ToVec(se3ToVecInput);

        double[][] adjointInput = Matrix.matrixMultiplication(Matrix.inverseMatrix(robot.X), robot.Xd);
        double[][] VdAdjointMatrix = Matrix.matrixMultiplication(Robotics.adjointMatrix(adjointInput), Matrix.transposeArray(Vd));
        return Matrix.flattenedMatrix(VdAdjointMatrix);
    }

    /**
     * Returns the commanded end-effector twist, using feedforward plus PI feedback control
     * @return V : Commanded end-effector twist, expressed in the end-effector frame
     */
    private double[] getCommandedTwist() {
        double[] xErr = getErrorTwist();
        // Error Integral is sum of all xErr * dT over time
        for (int i=0; i < xErr.length; i++) {
            robot.errorIntegral[i] += Matrix.scalarArrayMultiplication(xErr, robot.DELTA_T)[i];
        }
        double[][] KpError = Matrix.matrixMultiplication(robot.KpMatrix, Matrix.transposeArray(xErr));
        double[][] KiError = Matrix.matrixMultiplication(robot.KiMatrix, Matrix.transposeArray(robot.errorIntegral));
        double[] sumError = Matrix.arrayAddition(Matrix.flattenedMatrix(KpError), Matrix.flattenedMatrix(KiError));
        return Matrix.arrayAddition(getFeedForwardTwist(), sumError);
    }

    /**
     * Returns the controls needed for the commanded twist V
     * @return controls : 9-vector of controls indicating the 5 joint speeds Δθ / Δt and the 4 wheel speeds u
     */
    public double[] getControls() {
        double[][] Je = getFullJacobian();
        double[] V = getCommandedTwist();

        double[][] JePinv = Matrix.pseudoInverse(Je);
        double[][] controlsArray = Matrix.matrixMultiplication(JePinv, Matrix.transposeArray(V));
        double[] controls = Matrix.flattenedMatrix(controlsArray);

        // Check joint limits, and recalculate controls if needed
        NextState nextState = new NextState(robot.DELTA_T, robot.MAX_SPEED, robot.F);
        double[] nextConfig = nextState.getNextState(robot.currentConfig, controls);
        List<Integer> constrainJoints = testJointLimits(nextConfig);
        if (constrainJoints.size() > 0) {
            for (int joint : constrainJoints) {
                Matrix.replaceColumnValues(Je, joint - 1, 0);
                JePinv = Matrix.pseudoInverse(Je);
                controlsArray = Matrix.matrixMultiplication(JePinv, Matrix.transposeArray(V));
                controls = Matrix.flattenedMatrix(controlsArray);
            }
        }
        //nextState.getNextState(robot.currentConfig, controls);
        return controls;
    }
}
