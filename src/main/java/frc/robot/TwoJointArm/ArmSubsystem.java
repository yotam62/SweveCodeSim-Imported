package frc.robot.TwoJointArm;

import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.TwoJointArm.ArmIO.ArmIOinputs;
import frc.robot.TwoJointArm.ArmSim.SimState;
import frc.robot.TwoJointArm.KinematicsDynamics.jointXZ;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;


import org.littletonrobotics.junction.Logger;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.util.Units;


public class ArmSubsystem extends SubsystemBase {

    KinematicsDynamics kinematicsDynamics = new KinematicsDynamics();
     public ArmPointTraj desiredState = new ArmPointTraj(ArmConstants.shoulderStartingAngle, ArmConstants.elbowStartingAngle, 0, 0, 0, 0);
    private final Mechanism2d armMechanism = new Mechanism2d(6, 8);
    private final MechanismRoot2d root = armMechanism.getRoot("ShoulderJoint", 1.5, 2);
    private final MechanismLigament2d shoulderLigament =
        root.append(new MechanismLigament2d("shoulder", ArmConstants.shoulderLength, 0, 6, new Color8Bit(Color.RED)));
    private final MechanismLigament2d elbowLigament = shoulderLigament.append(new MechanismLigament2d("elbow", ArmConstants.elbowLength, 0, 6 ,new Color8Bit(Color.BLUE) ));

    jointXZ currentAngles = new jointXZ(0, 0);
    public record jointAngles(double x, double z) {}

    ArmIO io;
    ArmIOinputsAutoLogged inputs = new ArmIOinputsAutoLogged();


    public ArmSubsystem(ArmIO io) {
        this.io = io;

        SmartDashboard.putData("ArmMechanism", armMechanism);
         desiredState = new ArmPointTraj(ArmConstants.shoulderStartingAngle, ArmConstants.elbowStartingAngle, 0, 0, 0, 0);


    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ArmInputs", inputs);



        Logger.recordOutput("shoulderAngle", inputs.shoulderAngle);
        Logger.recordOutput("elbowAngle", inputs.elbowAngle);
        Logger.recordOutput("shoulderVelocity", inputs.shoulderVelocity);
        Logger.recordOutput("elbowVelocity", inputs.elbowVelocity);
        Logger.recordOutput("shoulderAppliedVoltage", inputs.shoulderAppliedVoltage);
        Logger.recordOutput("elbowAppliedVoltage", inputs.elbowAppliedVoltage);


        SmartDashboard.putNumber("elbowangle", inputs.elbowAngle);

        SmartDashboard.putNumber("DesiredshoulderVelocity", desiredState.shoulderAngularVelocity());
        SmartDashboard.putNumber("DesiredelbowVelocity", desiredState.elbowAngularVelocity());
        SmartDashboard.putNumber("DesiredshoulderAngle", desiredState.shoulderAngle());
        SmartDashboard.putNumber("DesiredelbowAngle", desiredState.elbowAngle());


        shoulderLigament.setAngle(Units.radiansToDegrees(inputs.shoulderAngle));
        elbowLigament.setAngle(Units.radiansToDegrees(inputs.elbowAngle));
    }

    public void setVoltage(double shoulderVolts, double elbowVolts) {
        io.setVoltage(shoulderVolts, elbowVolts);
    }

    public ArmPointTraj getcurrentState(){
        return new ArmPointTraj(
        inputs.shoulderAngle, inputs.elbowAngle,
        inputs.shoulderVelocity, inputs.elbowVelocity,
        0,0
        );
    }

    public ArmPointTraj createRandomArmPointTraj() {
        double x;
        double z;
        do{
        x = Math.random() * 2.5 -1; 
        z = Math.random() * 2.5 -1; } while ( Math.sqrt(x*x + z*z) > (ArmConstants.shoulderLength + ArmConstants.elbowLength) || Math.sqrt(x*x + z*z) < Math.abs(ArmConstants.shoulderLength - ArmConstants.elbowLength));
        jointXZ angles = KinematicsDynamics.getJointAngles(new ArmPose(x, z,0));
        return new ArmPointTraj(angles.shoulderAngle(), angles.elbowAngle(), 0, 0, 0, 0);

    }

    public void setDesiredState(ArmPointTraj desired) {
    this.desiredState = desired;
    }
}   
