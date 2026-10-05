package frc.robot.TwoJointArm;

import java.util.List;



public interface TrajGeneratorInterface {

    public ArmTraj createTraj(ArmPointTraj start, ArmPointTraj goal, List<Obstacle> obstacle);
        
}