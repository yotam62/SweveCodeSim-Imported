package frc.robot.TwoJointArm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOinputs{
        public double shoulderAngle;
        public double shoulderVelocity;
        public double shoulderAppliedVoltage;
        public double shoulderAppliedAmps;
        public double elbowAngle;
        public double elbowVelocity;    
        public double elbowAppliedVoltage;
        public double elbowAppliedAmps;

    }
    
    public default void updateInputs(ArmIOinputs inputs) {}

    public default void setVoltage(double shoulderAppliedVoltage, double elbowAppliedVoltage) {}
} 
