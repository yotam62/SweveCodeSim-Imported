package frc.robot.drive;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import static org.wpilib.units.Units.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.drive.VisionSubsystem.VisionMeasurement;

public class Drive extends SubsystemBase{

    static final double ODOMETRY_FREQUENCY = 150;
    private final Field2d m_field = new Field2d();
    public Rotation2d simRotation = new Rotation2d();
    public Rotation2d rawGyroRotation = new Rotation2d();
    Timer gyroResetTimer = new Timer();
    
    private Twist2d twist = new Twist2d();
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    

    SwerveModuleState[] mods = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()};

    public SwerveModulePosition[] lastModulePositions = // For delta tracking
    new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()};
    static final Lock odometryLock = new ReentrantLock();

    private SwerveDriveKinematics kinematics =  SwerveConstants.swerveKinematics;
    public SwerveDrivePoseEstimator SwervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.005,0.005, Radians.convertFrom(5, Degrees)), VecBuilder.fill(0.05, 0.05, 999999999) );


    public Drive(Module flModule, Module blModule, Module brModule, Module frModule) {  
        modules[0] = flModule;
        modules[1] = blModule;
        modules[2] = brModule;
        modules[3] = frModule;

        SmartDashboard.putData("field",m_field);
        gyroResetTimer.start();
    }
    public double getMaxLinearVelocityPerSecond() {
        return 5.0;
    }
    public double getMaxAngularSpeedRadPerSecond() {
        return getMaxLinearSpeedMetersPerSec() * 1 / SwerveConstants.DRIVE_BASE_RADIUS;
    }






    private boolean wasGyroDisconnected = false;
    SysIdRoutine routine;

    public Rotation2d getRotation() {
    return SwervePoseEstimator.getEstimatedPosition().getRotation();
    }   

    public void runVelocity(ChassisVelocities speeds) {

    ChassisVelocities discreteSpeeds = ChassisVelocities.discretize(speeds, 0.02); // making sure the robot will move correctly with the spin and speed we give it. ALWAYS use it.
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds); //turning the robot spin and speed into the spin and speed of each swerve module.
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec()); // making sure each sweerve module doesn't  go over the speed limit, could slow down the robot.
    for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
    }


 }

    public double getMaxLinearSpeedMetersPerSec() {
        return 5.0;
    }



    public void periodic(){

    for (var module : modules) {
        module.periodic();
    }


    double[] sampleTimestamps =
    modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
 
 
    for (int i = 0; i < sampleCount; i++) {
    // Read wheel positions and deltas from each module
    // double vx = 0;
    // double vy = 0;
    modulePositions = new SwerveModulePosition[4];
    moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
    modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];//make an array
    moduleDeltas[moduleIndex] =
    new SwerveModulePosition(
    modulePositions[moduleIndex].distanceMeters
    - lastModulePositions[moduleIndex].distanceMeters,
    modulePositions[moduleIndex].angle);
    lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }
    
    twist = kinematics.toTwist2d(moduleDeltas);
    

    // Use the angle delta from the kinematics and module deltas
    if (Gyro.connected) {
    // Use the real gyro angle
    rawGyroRotation = Gyro.odometryYawPositions[i];
    } else {
    // Use the angle delta from the kinematics and module deltas
    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
    
    
    

    
    //visionLock.lock();
    SwervePoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    // visionLock.unlock();
    }
 


    

         m_field.setRobotPose(SwervePoseEstimator.getEstimatedPosition());
        
        


        }
 


    public Pose2d getPose() {
        return SwervePoseEstimator.getEstimatedPosition();
    }  
    public void resetPosition(Pose2d pose) {
        SwervePoseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
    }

    public ChassisVelocities getChassisVelocities() {
        return kinematics.toChassisVelocities(getModuleStates());
    }

     public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
        modules[i].runCharacterization(output);
        }
    }

     /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
        output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }


    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        double totalSpeed = 0;
        for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
        totalSpeed += states[i].velocity;
        }
        
        return states;
    }

    public void addVision(VisionMeasurement measurement) {
    Vector<N3> stds = VecBuilder.fill(measurement.std()[0], measurement.std()[1], 9999999); /*  
 makes a "certainty vector" that is baisically how much we trust the vision measurement.
  //The first two numbers are the standard deviations of the x and y position, 
  and the last number is the standard deviation of the rotation. The lower the error, the more we trast vison, and vice versa */

 //visionLock.lock();
    SmartDashboard.putBoolean("angle conditions", Math.abs(Gyro.rollDegrees) < 1 || Math.abs(Gyro.pitchDegrees) < 1);
 if (Math.abs(Gyro.rollDegrees) < 5 && Math.abs(Gyro.pitchDegrees) < 5 && getGyroSpeed() < 180 && getTranslationalSpeed() < 5) {

 SwervePoseEstimator.addVisionMeasurement(new Pose2d(measurement.pose().getTranslation(), getRotation()), measurement.timestamp(), stds);

 if (gyroResetTimer.hasElapsed(15) && getGyroSpeed() < 1 && getTranslationalSpeed() < 0.1 && measurement.numTags() >= 2 && measurement.avgDistance() < 2.35) {
 SwervePoseEstimator.resetRotation(Rotation2d.fromDegrees(measurement.rotationDegreees()));
 //SmartDashboard.putBoolean("gyro reset", true);
 gyroResetTimer.restart();
 }
 /// SmartDashboard.putBoolean("gyro reset", false);

 

 }
 
 //visionLock.unlock();
 }

  public double getGyroSpeed() {
    return Math.abs(Units.radiansToDegrees(Gyro.yawVelocityRadPerSec));
 }

  public double getTranslationalSpeed() {
    return Math.hypot(getChassisVelocities().vx, getChassisVelocities().vy);
 }
  
 public void setPose(Pose2d pose) {
 odometryLock.lock();
 SwervePoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose); 

 odometryLock.unlock();
 }

  private SwerveModulePosition[] getModulePositions() {
 SwerveModulePosition[] states = new SwerveModulePosition[4];
 for (int i = 0; i < 4; i++) {
 states[i] = modules[i].getPosition();
 }
 return states;
 }

}