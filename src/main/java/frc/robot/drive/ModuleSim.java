package frc.robot.drive;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;

import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.simulation.DCMotorSim;
import frc.robot.Constants.SwerveConstants;

public class ModuleSim  implements Module {



        
        private static final DCMotor driveMotorModel = DCMotor.getKrakenX60(1); 
        private static final DCMotor turnMotorModel = DCMotor.getKrakenX60(1);
        private SwerveModulePosition[] odometryPosition = new SwerveModulePosition[1];
        private PIDController driveController = new PIDController(SwerveConstants.driveKP, 0, SwerveConstants.driveKD, 0.02);
        private PIDController turnController = new PIDController(SwerveConstants.angleKP, 0, 0, 0.02);
        private double driveFFVolts = 0;
        private double driveAppliedVolts = 0.0;
        private double turnAppliedVolts = 0.0;
        private final int index;
        private double[] odometryTimestamps = new double[]{0.0};
        
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAccelerationRadPerSec = 0.0;

          private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, SwerveConstants.driveReduction),
          driveMotorModel);
          private final DCMotorSim turnSim;



    public ModuleSim(int index, SwerveConstants.SwerveModuleConstants constants) {
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    // Set up turn sim (depends on index for correct reduction)
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                turnMotorModel,
                0.004,
                18.75),
            turnMotorModel);
    this.index = 0;
  }
    public void updateInputs() {
    // Run closed-loop control
   
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
   
      turnAppliedVolts = turnController.calculate(turnPosition.getRadians());
  
    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    drivePositionRad = driveSim.getAngularPositionRad();
    driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();


    turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
   // inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

    public void periodic() {

        updateInputs();
        odometryPosition = new SwerveModulePosition[1];
        odometryTimestamps[0] = Timer.getTimestamp();

        double positionMeters = drivePositionRad * SwerveConstants.WheelRadius ; 
        Rotation2d angle = turnAbsolutePosition;
        odometryPosition[0] = new SwerveModulePosition(positionMeters, angle);

    }

    public void runSetpoint(SwerveModuleVelocity state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());

        // Apply setpoints
        setDriveVelocity(state.velocity /SwerveConstants.WheelRadius);
        setTurnPosition(state.angle);
  }

  public void setDriveVelocity(double output) {
    driveController.setSetpoint(output);
    driveFFVolts = SwerveConstants.driveKS * Math.signum(output) + SwerveConstants.driveKV * output; //
  }

  public void setTurnPosition(Rotation2d rotation) {
    turnController.setSetpoint(rotation.getRadians());
  }



public double getPositionRadians() {
return drivePositionRad;
}


// @Override
// public void stop() {
// setDriveVelocity(0);
// }



public Rotation2d getAngle() {
return turnPosition;
}



public double getPositionMeters() {
    return drivePositionRad * SwerveConstants.WheelRadius;
}



public double getVelocityMetersPerSec() {
    return driveVelocityRadPerSec * SwerveConstants.WheelRadius;
}





public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
}



public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
}



public SwerveModulePosition[] getOdometryPositions() {
    return odometryPosition;
}

public double[] getOdometryTimestamps() {
    return odometryTimestamps;
}


public double getWheelRadiusCharacterizationPosition() {
return drivePositionRad;
}



public double getFFCharacterizationVelocity() {
return Units.radiansToRotations(driveVelocityRadPerSec);
}



public double getFFCharacterizationAcceleration() {
return Units.radiansToRotations(driveAccelerationRadPerSec);
}

}
