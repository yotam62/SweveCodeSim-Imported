package frc.robot.TwoJointArm;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.TwoJointArm.KinematicsDynamics.JointAccelerations;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.Nat;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.numbers.N4;
import org.wpilib.math.system.NumericalIntegration;


/*ok so the plan is this: In the big-ass function I take the current state's positions and speeds of the joints, 
and the forward dynamics (meaning if I plug x voltage what will the accel and speed and be, which I use the dynamics class and changes
based on mass matrix and shi like that). anyway I plug it all into the rk4 method that baysically calculates the accel and speed 
in 4 points instead of 1 and gives us a more detailed view of how the robot will act, and than the function spits out the 4 numbers:
 the new joint positions and speeds in the next point */
public class ArmSim implements ArmIO {

    public record SimState(double shoulderAngle, double elbowAngle, double shoulderVelocity, double elbowVelocity) {}

    
    private SimState currentState = new SimState(Constants.ArmConstants.shoulderStartingAngle, Constants.ArmConstants.elbowStartingAngle, 0, 0); // or whatever your real start angles are
    private double appliedShoulderVoltage = 0.0;
    private double appliedElbowVoltage = 0.0;



        public SimState getNextState(SimState previousState) {
            final int substeps = 10;
            final double subDt = 0.020 / substeps;

            SimState state = previousState;
            Matrix<N2, N1> u = VecBuilder.fill(appliedShoulderVoltage, appliedElbowVoltage);

            for (int i = 0; i < substeps; i++) {
                Matrix<N4, N1> x = VecBuilder.fill(
                    state.shoulderAngle(), state.elbowAngle(),
                    state.shoulderVelocity(), state.elbowVelocity());

                Matrix<N4, N1> next = NumericalIntegration.rk4(
            (Matrix<N4, N1> xIn, Matrix<N2, N1> uIn) -> {
                double shoulderAngle    = xIn.get(0, 0);
                double elbowAngle       = xIn.get(1, 0);
                double shoulderVelocity = xIn.get(2, 0);
                double elbowVelocity    = xIn.get(3, 0);

                double shoulderVoltage = uIn.get(0, 0);
                double elbowVoltage    = uIn.get(1, 0);

                double shoulderCurrent = ArmConstants.shoulderMotor.getCurrent(shoulderVelocity, shoulderVoltage);
                double elbowCurrent    = ArmConstants.elbowMotor.getCurrent(elbowVelocity, elbowVoltage);

                double shoulderTorque = ArmConstants.shoulderMotor.getTorque(shoulderCurrent);
                double elbowTorque    = ArmConstants.elbowMotor.getTorque(elbowCurrent);

                JointAccelerations accel = KinematicsDynamics.getAccelerations(
                    shoulderTorque, elbowTorque,
                    shoulderAngle, elbowAngle,
                    shoulderVelocity, elbowVelocity);

                return VecBuilder.fill(
                    shoulderVelocity, elbowVelocity,
                    accel.shoulderAcceleration(), accel.elbowAcceleration());
            }, x, u, subDt);

                state = new SimState(next.get(0,0), next.get(1,0), next.get(2,0), next.get(3,0));
            }
            return state;
        }
    

    @Override
    public void updateInputs(ArmIOinputs inputs) {
        currentState = getNextState(currentState);
        inputs.elbowAngle = currentState.elbowAngle();
        inputs.elbowVelocity = currentState.elbowVelocity();
        inputs.shoulderAngle = currentState.shoulderAngle();
        inputs.shoulderVelocity = currentState.shoulderVelocity();
        inputs.elbowAppliedVoltage = appliedElbowVoltage;
        inputs.shoulderAppliedVoltage = appliedShoulderVoltage;
        inputs.shoulderAppliedVoltage = appliedShoulderVoltage;
        inputs.elbowAppliedVoltage = appliedElbowVoltage;
        

    }

    @Override
    public void setVoltage(double shoulderVoltage, double elbowVoltage) {
        appliedShoulderVoltage = Math.max(Math.min(12, shoulderVoltage), -12);
        appliedElbowVoltage = Math.max(Math.min(12, elbowVoltage), -12);
    }

}
