package frc.robot.TwoJointArm;

import java.util.List;

public record ArmTraj (List<ArmPointTraj> waypoints, double duration, List<Double> timeStamps) {

   public ArmPointTraj sample(double time) {
        if (time <= 0) {
            return waypoints.get(0);
        } else if (time >= duration) {
            return waypoints.get(waypoints.size() - 1);
        } else {

            for(int i = 0; i < timeStamps.size() - 1; i ++) {
                if (timeStamps.get(i) == time){
                    return waypoints.get(i);
                }
                 
                if ((timeStamps.get(i) <= time) && (time <= timeStamps.get(i+1))){
                    ArmPointTraj a = waypoints.get(i);
                    ArmPointTraj b = waypoints.get(i + 1);
                    double tA = timeStamps.get(i);
                    double tB = timeStamps.get(i + 1);
                    double alpha = (time - tA) / (tB - tA);
                    double shoulderAngle = a.shoulderAngle() + alpha*(b.shoulderAngle()-a.shoulderAngle());
                    double elbowAngle =  a.elbowAngle() + alpha*(b.elbowAngle()-a.elbowAngle());
                    double elbowAngularVelocity = a.elbowAngularVelocity() + alpha* (b.elbowAngularVelocity() - a.elbowAngularVelocity());
                    double shoulderAngularVelocity = a.shoulderAngularVelocity() + alpha* (b.shoulderAngularVelocity() - a.shoulderAngularVelocity());
                    double shoulderAcceleration = (b.shoulderAngularVelocity() - a.shoulderAngularVelocity()) / (tB - tA);
                    double elbowAcceleration = (b.elbowAngularVelocity() - a.elbowAngularVelocity()) / (tB - tA);


                    return new ArmPointTraj(shoulderAngle, elbowAngle, shoulderAngularVelocity, elbowAngularVelocity, shoulderAcceleration, elbowAcceleration);

                }

            }

            throw new IllegalStateException("No bracketing waypoints found for time " + time);
        }

    
    }



}
