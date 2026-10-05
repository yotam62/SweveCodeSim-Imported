package frc.robot;

import org.wpilib.epilogue.Logged;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
  public static final Mode simMode = Mode.SIM;

  public static enum Mode {
    SIM,
    REAL
  }



  public static class SwerveConstants {

    public static final double driveReduction = 8.14; 
    public static final double steerReduction = 150/7;

    public static final double angleKP = 50;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKS = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 1; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.1996; // TODO: This must be tuned to specific robot
    public static final double driveKV = 0.6396;
    public static final double driveKA = 0.3;
    //public static final double driveKA = 0.3;

    public static final Slot0Configs intrinsicsD =
        new Slot0Configs().withKP(driveKP).withKD(driveKD).withKV(driveKV).withKS(driveKS).withKA(driveKA);
    public static final Slot0Configs instrinsicsS =
        new Slot0Configs().withKP(angleKP).withKD(angleKD).withKS(angleKS);
// TODO: see if I can delete this or not

    public record SwerveModuleConstants(
        int driveMotorID,
        int angleMotorID,
        int canCoderID,
        double angleOffset,
        boolean invertEncoder,
        boolean invertDrive,
        boolean invertSteer) {}

    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.10498;
      // 36.123046875 + 2.28515625)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 12;
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;

      public static final double angleOffset = -0.197998;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = 0.347656;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 6;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = false;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.343994;
      ;
      // -120.937
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    public static final int pigeonID = 13;
    public static final double trackWidth = 0.6275;
    public static final double wheelBase = 0.5835;
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //mod0 fl
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //mod1 bl
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // mod2 br
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0)); //mod3 fr

    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt(wheelBase * wheelBase / 4 + trackWidth * trackWidth / 4);
    //public static final double WheelRadius = 0.0508;
    public static final double WheelRadius = 0.049; // I don't have a carpet rn, this is just slightly lower than the real  radius of 0.0508
    public static final SwerveModuleConstants SwerveModuleConstants = null;

  }

  public class ArmConstants {

    //physical constants
    public static final double shoulderLength = 1; // meters
    public static final double shoulderMass = 5; // kilograms
    public static final double shoulderCGistance = 0.5; // meters
    public static final double shoulderMOI = 0.4174; // moment of inertia
    public static final double elbowLength = 0.5; // meters
    public static final double elbowMass = 2.5; // kilograms
    public static final double elbowCGistance = 0.25; // meters
    public static final double elbowMOI = 0.2807;// moment of inertia

    // Motor and gearbox constants
    public static final double shoulderGearRatio = 20; // gear ratio
    public static final int shoulderMotorID = 0; // motor ID
    public static final double elbowGearRatio = 15; // gear ratio
    public static final int elbowMotorID = 1; // motor ID

    // PID and feedforward gains
    public static final DCMotor shoulderMotor = DCMotor.getKrakenX60(1).withReduction(shoulderGearRatio); 
    public static final double shoulderKg = (shoulderMass * 9.81 * shoulderCGistance * shoulderMotor.rOhms) /  shoulderMotor.KtNMPerAmp; 
    public static final double shoulderKv = 1 / shoulderMotor.KvRadPerSecPerVolt; 
    public static final double shoulderKa = (shoulderMOI * shoulderMotor.rOhms) / shoulderMotor.KtNMPerAmp;

    public static final DCMotor elbowMotor = DCMotor.getKrakenX60(1).withReduction(elbowGearRatio); 
    public static final double elbowKg = (elbowMass * 9.81 * elbowCGistance * elbowMotor.rOhms) /elbowMotor.KtNMPerAmp ; 
    public static final double elbowKv = 1 / elbowMotor.KvRadPerSecPerVolt; 
    public static final double elbowKa = (elbowMOI * elbowMotor.rOhms) / elbowMotor.KtNMPerAmp;

    public static final LoggedTunableNumber shoulderAngleKP = new LoggedTunableNumber("Arm/Shoulder/Angle/KP", 0);
    public static final double shoulderAngleKI = 0; 
    public static final LoggedTunableNumber shoulderAngleKD = new LoggedTunableNumber("Arm/Shoulder/Angle/KD", 2);  

    public static final LoggedTunableNumber shoulderVelocityKP = new LoggedTunableNumber("Arm/Shoulder/Velocity/KP", 0);
    public static final double shoulderVelocityKI = 0;
    public static final LoggedTunableNumber shoulderVelocityKD = new LoggedTunableNumber("Arm/Shoulder/Velocity/KD", 0);

    public static final LoggedTunableNumber elbowAngleKP = new LoggedTunableNumber("Arm/Elbow/Angle/KP", 0);
    public static final double elbowAngleKI = 0; 
    public static final LoggedTunableNumber elbowAngleKD = new LoggedTunableNumber("Arm/Elbow/Angle/KD", 0.3);

    public static final LoggedTunableNumber elbowVelocityKP = new LoggedTunableNumber("Arm/Elbow/Velocity/KP", 0.1);
    public static final double elbowVelocityKI = 0;
    public static final LoggedTunableNumber elbowVelocityKD = new LoggedTunableNumber("Arm/Elbow/Velocity/KD", 0);

    //TODO: add current limits
    public static final double shoulderMaxVelocity = 2.5; //rad/sec
    public static final double shoulderMaxAcceleration = 6; //rad/sec^2

    public static final double shoulderMaxDeceleration = 6; //rad/sec^2
    public static final double elbowMaxDeceleration = 25; //rad/sec^2

    public static final double elbowMaxVelocity = 4; //rad/sec
    public static final double elbowMaxAcceleration = 25; //rad/sec^2

    public static final double dt = 0.02; // seconds
    public static final double PointsPerPath = 25; // number of points in the trajectory

    public static final double shoulderStartingAngle = 0; // radians
    public static final double elbowStartingAngle = 0; // radians

  }


}
