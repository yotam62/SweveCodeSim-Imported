package frc.robot.TwoJointArm.ArmCommands;

import org.wpilib.command2.Command;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.ArmSubsystem;
import frc.robot.TwoJointArm.ArmTraj;
import frc.robot.TwoJointArm.KinematicsDynamics;

public class StayCommand extends Command {

    KinematicsDynamics kinematics = new KinematicsDynamics();
    private ArmSubsystem Arm;
    ArmPointTraj staticState;
    ArmPointTraj currentState;


        public StayCommand(ArmSubsystem Arm) {
        this.Arm = Arm;

        addRequirements(Arm);
    }

    public void initialize() {
        staticState = new ArmPointTraj(Arm.getcurrentState().shoulderAngle(), Arm.getcurrentState().elbowAngle(), 0, 0, 0, 0);
        Arm.setDesiredState(staticState);
    }

    public void execute() {
        currentState = new ArmPointTraj(Arm.getcurrentState().shoulderAngle(), Arm.getcurrentState().elbowAngle(), 0,0,0,0);


        KinematicsDynamics.JointVoltages jointVoltages = kinematics.getNeededVoltage(staticState);
        KinematicsDynamics.JointVoltages jointVoltagesPID = kinematics.addPID(jointVoltages, staticState, Arm.getcurrentState());
        double shoulderVoltagePID = jointVoltagesPID.shoulderVoltage();
        double elbowVoltagePID = jointVoltagesPID.elbowVoltage();

        Arm.setVoltage(shoulderVoltagePID, elbowVoltagePID);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Arm.setVoltage(0, 0);

    }
    
}
