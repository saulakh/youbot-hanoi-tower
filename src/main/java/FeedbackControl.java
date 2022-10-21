import libraries.Matrix;
import libraries.Robotics;

import java.util.ArrayList;
import java.util.List;

public class FeedbackControl {

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
        double[][] Te0 = Matrix.inverseMatrix(T0e);
        double[][] Te0Adjoint = Robotics.adjointMatrix(Te0);

        // Build F6 matrix
        int m = F[0].length;
        double[][] F6Matrix = new double[6][m];
        Matrix.replaceRangeFromMatrix(F, F6Matrix, 2, 0);

        // Get full Jacobian matrix
        double[][] baseJacobian = Matrix.matrixMultiplication(Te0Adjoint, F6Matrix);
        double[][] armJacobian = Robotics.jacobianBody(BList, thetaList);

        // Concatenate [armJacobian, baseJacobian] to match order of controls and config
        double[][] fullJacobian = new double[6][baseJacobian[0].length + armJacobian[0].length];
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

    public double[] feedbackControl(YouBot robot, double[][] X, double[][] Xd, double[][] XdNext, double[][] KpMatrix, double[][] KiMatrix, double dT, double[] currentConfig, double[] errorIntegral) {
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
    - controls: 5 joint speeds and 4 wheel speeds needed to reach next position
     */
        double[] thetaList = new double[] {0,0,0.2,-1.6,0};
        // Get end-effector configuration relative to base frame (for test input angles)
        double[][] T0e = Robotics.fkInBody(robot.M0e, robot.BList, thetaList);
        double[][] Je = jacobian(T0e, robot.F, robot.BList, thetaList);

        double[][] xInv = Matrix.inverseMatrix(X);
        double[][] XdInv = Matrix.inverseMatrix(Xd);

        // Error twist between current and reference state
        double[][] matrixLog6Input = Matrix.matrixMultiplication(xInv, Xd);
        double[][] se3ToVecInput = Robotics.matrixLog6(matrixLog6Input);
        double[] xErr = Robotics.se3ToVec(se3ToVecInput);

        // Error Integral is sum of all xErr * dT over time
        for (int i=0; i < xErr.length; i++) {
            errorIntegral[i] += Matrix.scalarArrayMultiplication(xErr, dT)[i];
        }
        robot.errorIntegral = errorIntegral;

        // Feedforward reference twist
        matrixLog6Input = Matrix.matrixMultiplication(XdInv, XdNext);
        se3ToVecInput = Matrix.scalarMultiplication(matrixLog6Input, 1/dT);
        double[] Vd = Robotics.se3ToVec(se3ToVecInput);
        double[][] adjointInput = Matrix.matrixMultiplication(xInv, Xd);
        double[][] VdAdjointMatrix = Matrix.matrixMultiplication(Robotics.adjointMatrix(adjointInput), Matrix.transposeArray(Vd));
        double[] VdAdjoint = Matrix.flattenedMatrix(VdAdjointMatrix);

        // Get commanded end-effector twist V
        double[][] KpError = Matrix.matrixMultiplication(KpMatrix, Matrix.transposeArray(xErr));
        double[][] KiError = Matrix.matrixMultiplication(KiMatrix, Matrix.transposeArray(errorIntegral));
        double[] sumError = Matrix.arrayAddition(Matrix.flattenedMatrix(KpError), Matrix.flattenedMatrix(KiError));
        double[] V = Matrix.arrayAddition(VdAdjoint, sumError);

        // Get controls and test joint limits
        double[][] JeInv = Matrix.pseudoInverse(Je);
        double[][] controlsArray = Matrix.matrixMultiplication(JeInv, Matrix.transposeArray(V));
        double[] controls = Matrix.flattenedMatrix(controlsArray);

        // Check joint limits, and recalculate controls if needed
        double[] nextConfig = NextState.nextState(currentConfig, controls, dT, robot.MAX_SPEED);
        List<Integer> constrainJoints = testJointLimits(nextConfig, 2);
        double tolerance = 0.002;
        // TODO: Debug this part, doesn't seem to be constraining joints correctly
        if (constrainJoints.size() > 0) {
            for (int joint : constrainJoints) {
                Matrix.replaceColumnValues(Je, joint - 1, 0);
                double[][] JePinv = Matrix.pseudoInvTol(Je, tolerance);
                controlsArray = Matrix.matrixMultiplication(JePinv, Matrix.transposeArray(V));
                controls = Matrix.flattenedMatrix(controlsArray);
            }
        }

        return controls;
    }
}
