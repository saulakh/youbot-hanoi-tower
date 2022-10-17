import matrix.Matrix;

import java.util.ArrayList;
import java.util.List;

public class FeedbackControl {

    YouBot robot = new YouBot();

    public double[][] jacobian(double[][] T0e, double[][] F, double[][] BList, double[] thetaList) {
        /*
        Inputs:
        - T0e: transformation matrix from base to end-effector
        - F: psuedoinverse of chassis kinematic model H(0)
        - BList: screw axes when arm is at home configuration
        - thetaList: list of 5 joint angles for arm configuration
        Output:
        - fullJacobian: Full Jacobian matrix (armJacobian, baseJacobian) to match order of controls and config
         */
        // TODO: Finish debugging this method
        double[][] Te0 = Matrix.inverseMatrix(T0e);
        double[][] Te0Adjoint = robot.adjointMatrix(Te0);

        // Build F6 matrix
        int m = F[0].length;
        double[][] F6Matrix = new double[6][m];
        Matrix.replaceRangeFromMatrix(F, F6Matrix, 2, 0);

        // Get full Jacobian matrix
        double[][] baseJacobian = Matrix.matrixMultiplication(Te0Adjoint, F6Matrix);
        double[][] armJacobian = robot.jacobianBody(BList, thetaList);

        // Concatenate [armJacobian, baseJacobian] to match order of controls and config
        double[][] fullJacobian = new double[6][9]; // TODO: don't hardcode dimensions
        Matrix.replaceRangeFromMatrix(armJacobian, fullJacobian, 0, 0);
        Matrix.replaceRangeFromMatrix(baseJacobian, fullJacobian, 0, armJacobian[0].length);

        return fullJacobian;
    }

    public List<Integer> testJointLimits(double[] currentConfig, double jointMax) {
        /*
        Returns a list of joint angles that exceed joint limits
         */
        List<Integer> constrainJoints = new ArrayList<>();
        double theta3 = currentConfig[5];
        double theta4 = currentConfig[6];
        if (theta3 < -jointMax || theta3 > jointMax) {
            constrainJoints.add(3);
        }
        if (theta4 < -jointMax || theta4 > jointMax) {
            constrainJoints.add(4);
        }
        return constrainJoints;
    }

    public double[] feedbackControl(double[][] X, double[][] Xd, double[][] XdNext, double[][] KpMatrix, double[][] KiMatrix, double dT, double[] currentConfig, double[] errorIntegral) {
    /*
    Inputs:
    - X: current actual end-effector configuration (Tse)
    - Xd: current end-effector reference configuration Xd (Tse_d)
    - XdNext: reference configuration at next timestep in ref trajectory
    - KpMatrix, KiMatrix: PI gain matrices
    - dT: timestep between ref trajectory configurations
    - currentConfig: (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4)
    - errorIntegral: sum of all error twists over time
    Outputs:
    - V: commanded end-effector twist, expressed in end-effector frame
     */
        double[] V = new double[6];
        return V;
    }
}
