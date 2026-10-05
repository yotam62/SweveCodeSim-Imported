package frc.robot.drive;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.GamepadButton;

/*The plan:
 * Get the robot current location, the alliance color and the hub location in feild coordinates. 
 * make a vector that goes from the robot position to the hub position
 * make a  rotation2d object that is the  diraction we need to face (not nececerily rotation 2d)
 * make a chassisSpeeds object and also optimize the rotation.
 * check by the DriveCommand what needs to be done here and what willl happen in the drive class
 */

public class AutoAlignHubCommand extends Command {

    private Drive swerve;
    private DoubleSupplier rotationSupplier;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    boolean isAllianceRed;
    private boolean isHubLocationKnown = false;
    Gamepad controller;
    Translation2d robotLocation;
    Translation2d Hublocation;
    private double rotationKP = 8;
    double omega;

    
     
    public AutoAlignHubCommand(Drive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Gamepad controller) {
        this.swerve = swerve;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
            addRequirements(swerve);
    }


    public void initialize() {

        if(MatchState.getAlliance().isPresent()){
            boolean isAllianceRed = MatchState.getAlliance().isPresent() 
            && MatchState.getAlliance().get() == Alliance.RED;
            isHubLocationKnown = true;
            Hublocation = getHubLocation(isAllianceRed);
        }
        
        else{
            isHubLocationKnown = false;
        }

    }
    public void execute() {


        if(!isHubLocationKnown) return ;
        
        if(!isHubLocationKnown && MatchState.getAlliance().isPresent()){
            Hublocation = getHubLocation(isAllianceRed);
            isAllianceRed = MatchState.getAlliance().get() == Alliance.RED;
            isHubLocationKnown = true;

        }


        calculateOmegaToHub(swerve.getPose().getTranslation(), swerve);

        Translation2d linearVelocity;
        linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        if(omega > swerve.getMaxAngularSpeedRadPerSecond()){
            omega = swerve.getMaxAngularSpeedRadPerSecond();
        }
        else if(omega < -swerve.getMaxAngularSpeedRadPerSecond()){
            omega = -swerve.getMaxAngularSpeedRadPerSecond();
        }

        boolean isFlipped =
        MatchState.getAlliance().isPresent()
        && MatchState.getAlliance().get() == Alliance.RED;
 
        ChassisVelocities speeds = new ChassisVelocities(
        linearVelocity.getX() * swerve.getMaxLinearVelocityPerSecond(),
        linearVelocity.getY() * swerve.getMaxLinearVelocityPerSecond(),
        omega);

        swerve.runVelocity(
        ChassisVelocities.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                : swerve.getRotation()));


    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) { 
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
    public Translation2d getHubLocation(boolean isAllianceRed){ 
        if(isAllianceRed){
            return new Translation2d(11.813388,4.034);
        } 
        else {
            return new Translation2d(4.626, 4.034);
        }
    }

        public double calculateOmegaToHub(Translation2d robotLocation, Drive swerve){
        robotLocation = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
        Translation2d rotationVector = Hublocation.minus(robotLocation);
        Rotation2d allignDirection = rotationVector.getAngle();
        double error = allignDirection.minus(swerve.SwervePoseEstimator.getEstimatedPosition().getRotation()).getRadians();
        double radianDistance = MathUtil.angleModulus(error);
        double omega = radianDistance * rotationKP; 
        return omega;

        }


    
} 

