import java.util.ArrayList;
import java.util.List;

public class FeedbackControl {

    public static double[][] jacobian(double[][] T0e, double[][] F, double[][] BList, double[][] thetaList) {
        /*
        Inputs:
        - T0e: transformation matrix from base to end-effector
        - F: psuedoinverse of chassis kinematic model H(0)
        - BList: screw axes when arm is at home configuration
        - thetaList: list of 5 joint angles for arm configuration
        Output:
        - fullJacobian: Full Jacobian matrix (J_arm, J_base) to match order of controls and config
         */
        double[][] fullJacobian = new double[6][9];
        // Need to create method for Adjoint representation of Teb

        // Build F6 matrix

        // baseJacobian is AdjointTeb @ F6
        // Need to create JacobianBody method to calculate armJacobian

        return fullJacobian;
    }

    public static List<Double> testJointLimits(double[] currentConfig) {
        /*
        Returns a list of joint angles that exceed joint limits
         */
        List<Double> constrainJoints = new ArrayList<>();
        return constrainJoints;
    }

    public static double[][] pseudoInvTol(double[][] matrix, double tolerance) {
        /*
        Replaces any values greater than the tolerance with the tolerance, and returns the pseudo inverse of the matrix
         */
        return matrix;
    }

    public static double[] feedbackControl(double[][] X, double[][] Xd, double[][] XdNext, double[][] KpMatrix, double[][] KiMatrix, double dT, double[] currentConfig, double[] errorIntegral) {
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
