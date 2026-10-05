/*make the motors and all the vars I need, make the constructor, the turn and drive motors. make an update input method and in the periodic run it. make a runsetpoint method */
package frc.robot.drive;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import frc.robot.Constants.SwerveConstants;

public class ModuleReal implements Module {

    double[] odometryTimestamps = new double[] {};  
  double[] odometryDrivePositionsRad = new double[] {};  
  Rotation2d[] odometryTurnPositions = new Rotation2d[] {};  


    
    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

     // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<AngularAcceleration> driveAcceleration;
 

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;

  TalonFXConfiguration driveConfig;
  TalonFXConfiguration turnConfig;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);


    int sampleCount = 0;
    private final int index;
    private final SwerveConstants.SwerveModuleConstants constants;

    private boolean wasDisconnectedDrive = false;
    private boolean wasDisconnectedTurn = false;
    private boolean wasDisconnectedTurnEncoder = false;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    public ModuleReal (SwerveConstants.SwerveModuleConstants constants, int index) {
        this.constants = constants;
        this.index = index;


        driveTalon = new TalonFX(constants.driveMotorID(), "rio");
        turnTalon = new TalonFX(constants.angleMotorID(), "rio");
        cancoder = new CANcoder(constants.canCoderID(), "rio");

        driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = SwerveConstants.intrinsicsD;
        driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveReduction;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
        driveConfig.CurrentLimits.StatorCurrentLimit = 60; 
            
        driveConfig.MotorOutput.Inverted =
        constants.invertDrive()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        driveTalon.getConfigurator().apply(driveConfig, 0.25);
        driveTalon.setPosition(0.0, 0.25);





    // Configure CANCoder
     CANcoderConfiguration config = new CANcoderConfiguration();
     config.MagnetSensor.SensorDirection =
         constants.invertEncoder()
             ? SensorDirectionValue.Clockwise_Positive
             : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = constants.angleOffset();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    cancoder.getConfigurator().apply(config, 0.25);


    // Configure turn motor
    turnConfig = new TalonFXConfiguration();

    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = SwerveConstants.instrinsicsS;

    turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turnConfig.CurrentLimits.StatorCurrentLimit = 40;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.canCoderID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    


    //turnConfig.Feedback.RotorToSensorRatio = 1;
    //turnConfig.Feedback.SensorToMechanismRatio = 18.75;
    turnConfig.Feedback.RotorToSensorRatio = 18.75;
    turnConfig.Feedback.SensorToMechanismRatio = 1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =

    // Create turn status signals
        constants.invertSteer()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

                turnTalon.getConfigurator().apply(turnConfig, 0.25);
   
     //driveTalon.setInverted(constants.invertSteer());

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAcceleration = driveTalon.getAcceleration();
   // driveAppliedVolts = driveTalon.getMotorVoltage();
   // driveCurrent = driveTalon.getStatorCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();

    //turnAppliedVolts = turnTalon.getMotorVoltage();
   // turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAcceleration,
     
        turnAbsolutePosition,
        turnVelocity);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  // @Override
  // public void updateInputs(ModuleIOInputs inputs) {
  //   // Refresh all signals
   
  //   var driveStatus =
  //       BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAcceleration);
  //   var turnStatus =
  //       BaseStatusSignal.refreshAll(turnPosition, turnAbsolutePosition);
  //   var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

  //   inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
  //   inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
  //   inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    
    

  //   // Update drive inputs
   
  //   inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
  //   inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
  //   inputs.driveAccelerationRadPerSec = Units.rotationsToRadians(driveAcceleration.getValueAsDouble());
  //   // inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
  //   // inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

  //   // Update turn inputs
  //   inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
  //   inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
  //   // inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
  //   // inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
  //   // inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

  //   // Update odometry inputs
  //   inputs.odometryTimestamps =
  //       timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
  //   inputs.odometryDrivePositionsRad =
  //       drivePositionQueue.stream()
  //           .mapToDouble((Double value) -> Units.rotationsToRadians(value))
  //           .toArray();
  //   inputs.odometryTurnPositions =
  //       turnPositionQueue.stream()
  //           .map((Double value) -> Rotation2d.fromRotations(value))
  //           .toArray(Rotation2d[]::new);
  //   timestampQueue.clear();
  //   drivePositionQueue.clear();
  //   turnPositionQueue.clear();

  //   // SmartDashboard.putNumber(
  //   //     "encoder val internal for" + index, turnTalon.getPosition().getValueAsDouble());
  //   // SmartDashboard.putNumber("encoder for" + index, cancoder.getPosition().getValueAsDouble());
  //   // SmartDashboard.putNumber(
  //   //     "encoder val internal for drive for" + index, driveTalon.getPosition().getValueAsDouble());
  // }

  public void periodic() {

    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAcceleration, turnPosition, turnAbsolutePosition);

    odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    odometryDrivePositionsRad = drivePositionQueue.stream()
    .mapToDouble((Double value) -> Units.rotationsToRadians(value)).toArray();
    odometryTurnPositions = turnPositionQueue.stream()
    .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);


    sampleCount = odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = odometryDrivePositionsRad[i] * SwerveConstants.WheelRadius;
      Rotation2d angle = odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
    

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }
 
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output).withEnableFOC(false));
  }


  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(voltageRequest.withOutput(output).withEnableFOC(false));
  }




  public void lowerCurrentLimits() {
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
    driveConfig.CurrentLimits.StatorCurrentLimit = 80;
    driveTalon.getConfigurator().apply(driveConfig, 0.25);

    turnConfig.CurrentLimits.SupplyCurrentLimit = 30;
    turnConfig.CurrentLimits.StatorCurrentLimit = 40;
    turnTalon.getConfigurator().apply(turnConfig, 0.25);


    }



public double getPositionRadians() {
    return Units.rotationsToRadians(drivePosition.getValueAsDouble());
}

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
  
    driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec).withEnableFOC(false));
  }
    
    /** Disables all outputs to motors. */
    public void stop() {
      driveTalon.setControl(voltageRequest.withOutput(0));
      turnTalon.setControl(voltageRequest.withOutput(0));
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle(){
      return Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle()); //makes sure we don't go the long way
    //state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    setDriveVelocity(state.velocity /SwerveConstants.WheelRadius);
    setTurnPosition(state.angle);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return getPositionRadians() * SwerveConstants.WheelRadius;
  }
    
  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return Units.rotationsToRadians(driveVelocity.getValueAsDouble()) * SwerveConstants.WheelRadius;
  }

    /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

    /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

    /** Returns the module positions received this cycle. */
  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }


  public double[] getOdometryTimestamps() {
    return odometryTimestamps;
  }

    /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return getPositionRadians() ;
  }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(getVelocityMetersPerSec() / SwerveConstants.WheelRadius);
  }

  public double getFFCharacterizationAcceleration() {
    return Units.radiansToRotations(driveAcceleration.getValueAsDouble());
  }

    /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    setDriveOpenLoop(output);
    setTurnPosition(new Rotation2d());
  }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()).withEnableFOC(true));
    }
  }




