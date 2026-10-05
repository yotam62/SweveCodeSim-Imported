package frc.robot.TwoJointArm;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;

public class TrapezoidalTrajGenerator implements TrajGeneratorInterface {

    // Everything needed to sample one joint's motion at any time t.
    private record JointProfile(
        double startAngle,
        double goalAngle,
        double direction,
        double maxVelocity,
        double maxAcceleration,
        double maxDeceleration,
        double accelTime,
        double cruiseTime,
        double decelTime,
        double totalTime
    ) {}

    private record JointState(double angle, double velocity, double acceleration) {}

    private JointProfile computeProfile(double start, double goal, double maxVel, double maxAccel, double maxDecel) {
        double distance = Math.abs(goal - start);
        double direction = Math.signum(goal - start);

        double accelDistance = (maxVel * maxVel) / (2 * maxAccel);
        double decelDistance = (maxVel * maxVel) / (2 * maxDecel);

        double accelTime;
        double decelTime;
        double cruiseTime;

        if (accelDistance + decelDistance > distance) {

            maxVel = Math.sqrt((2 * distance * maxAccel * maxDecel) / (maxAccel + maxDecel));
            accelTime = maxVel / maxAccel;
            decelTime = maxVel / maxDecel;
            cruiseTime = 0;
        } else {
            double cruiseDistance = distance - accelDistance - decelDistance;
            accelTime = maxVel / maxAccel;
            decelTime = maxVel / maxDecel;
            cruiseTime = cruiseDistance / maxVel;
        }

        double totalTime = accelTime + cruiseTime + decelTime;

        return new JointProfile(start, goal, direction, maxVel, maxAccel, maxDecel, accelTime, cruiseTime, decelTime, totalTime);
    }

    private JointState sampleJoint(JointProfile p, double t) {
        t = Math.min(t, p.totalTime());

        if (t < p.accelTime()) {
            double angle = p.startAngle() + p.direction() * 0.5 * p.maxAcceleration() * t * t;
            double velocity = p.direction() * p.maxAcceleration() * t;
            double acceleration = p.direction() * p.maxAcceleration();
            return new JointState(angle, velocity, acceleration);
        }

        if (t < p.accelTime() + p.cruiseTime()) {
            double accelDistance = 0.5 * p.maxAcceleration() * p.accelTime() * p.accelTime();
            double angle = p.startAngle() + p.direction() * (accelDistance + p.maxVelocity() * (t - p.accelTime()));
            double velocity = p.direction() * p.maxVelocity();
            return new JointState(angle, velocity, 0);
        }

        double timeRemaining = p.totalTime() - t;
        double angle = p.goalAngle() - p.direction() * 0.5 * p.maxDeceleration() * timeRemaining * timeRemaining;
        double velocity = p.direction() * p.maxDeceleration() * timeRemaining;
        double acceleration = -p.direction() * p.maxDeceleration();
        if (t >= p.totalTime()){
             return new JointState(p.goalAngle(), 0, 0);}
        return new JointState(angle, velocity, acceleration);
        
    }

@Override
public ArmTraj createTraj(ArmPointTraj start, ArmPointTraj goal, List<Obstacle> obstacle) {
    double dS = goal.shoulderAngle() - start.shoulderAngle();
    double dE = goal.elbowAngle() - start.elbowAngle();

    // Tightest limit across joints, expressed per unit of path progress
    double vLim = Double.MAX_VALUE, aLim = Double.MAX_VALUE, dLim = Double.MAX_VALUE;
    if (Math.abs(dS) > 1e-9) {
        vLim = Math.min(vLim, Constants.ArmConstants.shoulderMaxVelocity / Math.abs(dS));
        aLim = Math.min(aLim, Constants.ArmConstants.shoulderMaxAcceleration / Math.abs(dS));
        dLim = Math.min(dLim, Constants.ArmConstants.shoulderMaxDeceleration / Math.abs(dS));
    }
    if (Math.abs(dE) > 1e-9) {
        vLim = Math.min(vLim, Constants.ArmConstants.elbowMaxVelocity / Math.abs(dE));
        aLim = Math.min(aLim, Constants.ArmConstants.elbowMaxAcceleration / Math.abs(dE));
        dLim = Math.min(dLim, Constants.ArmConstants.elbowMaxDeceleration / Math.abs(dE));
    }
    if (vLim == Double.MAX_VALUE) { // nothing to move
        return new ArmTraj(List.of(goal), 0, List.of(0.0));
    }

    
    JointProfile p = computeProfile(0, 1, vLim, aLim, dLim);

    List<ArmPointTraj> pointList = new ArrayList<>();
    List<Double> timeStamps = new ArrayList<>();
    for (double t = 0; t < p.totalTime(); t += Constants.ArmConstants.dt) {
        JointState s = sampleJoint(p, t);
        pointList.add(new ArmPointTraj(
            start.shoulderAngle() + dS * s.angle(), start.elbowAngle() + dE * s.angle(),
            dS * s.velocity(), dE * s.velocity(),
            dS * s.acceleration(), dE * s.acceleration()));
        timeStamps.add(t);
    }
    pointList.add(new ArmPointTraj(goal.shoulderAngle(), goal.elbowAngle(), 0, 0, 0, 0));
    timeStamps.add(p.totalTime());
    return new ArmTraj(pointList, p.totalTime(), timeStamps);
}
}