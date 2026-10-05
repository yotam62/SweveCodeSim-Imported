package frc.robot.drive;

import org.wpilib.command2.Command;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;

public class ShootingPathsAuto extends Command{

    Drive swerve;
    Timer timer = new Timer();
    double TimeToFinish = 4.25;
    Translation2d velocityVector;
    Translation2d targetPosition;
    double rotationKP = 8;

    public ShootingPathsAuto(Drive swerve ,Translation2d targetPosition) {
        this.swerve = swerve;
        this.targetPosition = targetPosition;
        addRequirements(swerve);
    }

    public void initialize() {
        timer.restart();

        Translation2d currentPos = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
       
        Translation2d distance = targetPosition.minus(currentPos);
        double speed = currentPos.getDistance(targetPosition) / TimeToFinish;
        if(speed > 3) {
            speed = 3;
        }
        if(distance.getNorm() > 0.01) {
            velocityVector = distance.times(speed/distance.getNorm()); // normalize the distance vector and then scale it to the desired speed
        }
        else {
            velocityVector = new Translation2d();
        }

    }

    public void execute() {
        boolean isAllianceRed = MatchState.getAlliance().isPresent() 
        && MatchState.getAlliance().get() == Alliance.RED;
        Translation2d robotLocation = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
        Translation2d rotationVector = getHubLocation(isAllianceRed).minus(robotLocation);
        Rotation2d allignDirection = rotationVector.getAngle(); //all that shit just for allign direction

        Rotation2d currentAngle = swerve.SwervePoseEstimator.getEstimatedPosition().getRotation(); //just getting the omega (rotation)
        Rotation2d wantedAngle = allignDirection;    
        Rotation2d currentToWanted = wantedAngle.minus(currentAngle);
        double RadianDistance = currentToWanted.getRadians();
        RadianDistance = MathUtil.angleModulus(RadianDistance);
        double omega = RadianDistance*rotationKP;


        ChassisVelocities speeds = new ChassisVelocities
        (velocityVector.getX(),
        velocityVector.getY(),
        omega);
        boolean isFlipped =
        MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED;

        swerve.runVelocity(
                  ChassisVelocities.fromFieldRelativeSpeeds(
                      speeds,
                      swerve.getRotation()));


    }

    public boolean isFinished() {
        return timer.hasElapsed(TimeToFinish+0.5);

    }
    public void end(boolean interupted){
        swerve.runVelocity(new ChassisVelocities());

    }
    public Translation2d getHubLocation(boolean isAllianceRed){ 
        if(isAllianceRed){
            return new Translation2d(11.813388,4.034);
        } 
        else {
            return new Translation2d(4.626, 4.034);
        }
    }
}
