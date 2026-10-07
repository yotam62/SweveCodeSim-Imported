 package frc.robot.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public interface Module {


        // public boolean driveConnected = false;
        // public double drivePositionRad = 0.0;
        // public double driveVelocityRadPerSec = 0.0;
        // public double driveAccelerationRadPerSec = 0.0;
        // // public double driveAppliedVolts = 0.0;
        // // public double driveCurrentAmps = 0.0;

        // public boolean turnConnected = false;
        // public boolean turnEncoderConnected = false;
        // public Rotation2d turnAbsolutePosition = new Rotation2d();
        // public Rotation2d turnPosition = new Rotation2d();
        // //  public double turnVelocityRadPerSec = 0.0;
        // //  public double turnAppliedVolts = 0.0;
        // // public double turnCurrentAmps = 0.0;

      public static final double[] odometryTimestamps = new double[] {0.0};
        // public double[] odometryDrivePositionsRad = new double[] {};
        // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
        
    
    /** Runs the module with the specified output while controlling to zero degrees. */
    public default void runCharacterization(double output) {}

    public default void periodic(){}

    public double getPositionRadians();

    public default void setDriveVelocity(double output){}

    public default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    public default void setTurnPosition(Rotation2d rotation) {}
    
    /** Disables all outputs to motors. */
    public default void stop() {}

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle();

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public default void runSetpoint(SwerveModuleVelocity state){}

      /** Returns the current drive position of the module in meters. */
    public double getPositionMeters();

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec();

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition();

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleVelocity getVelocity();

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions();

    public double[] getOdometryTimestamps();

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition();

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity();

    public double getFFCharacterizationAcceleration();
 

}