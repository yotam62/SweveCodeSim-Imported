package frc.robot.TwoJointArm;

public record ArmPointTraj (
    double shoulderAngle, double elbowAngle, 
    double shoulderAngularVelocity, double elbowAngularVelocity,
    double shoulderAcceleration, double elbowAcceleration) {}
