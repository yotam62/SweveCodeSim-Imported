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
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.GamepadButton;

public class DriveCommand extends Command {
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private Drive swerve;
    private DoubleSupplier rotationSupplier;
    CommandGamepad controller;
    
      public DriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Drive swerve, CommandGamepad controller) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.swerve = swerve;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        addRequirements(swerve);


    }

    public void execute() {
        Translation2d linearVelocity;
       // if(controller.isConnected()){            
            linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());



        double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);
 
        omega = Math.copySign(omega * omega, omega);
        boolean isFlipped =
            MatchState.getAlliance().isPresent()
            && MatchState.getAlliance().get() == Alliance.BLUE;

        ChassisVelocities speeds = new ChassisVelocities(
         linearVelocity.getX() * swerve.getMaxLinearVelocityPerSecond(),
         linearVelocity.getY() * swerve.getMaxLinearVelocityPerSecond(),
         omega* swerve.getMaxAngularSpeedRadPerSecond());

        Rotation2d rootangle = isFlipped
            ? swerve.getRotation().plus(new Rotation2d(Math.PI))
            : swerve.getRotation();
        swerve.runVelocity(speeds.toRobotRelative(rootangle));

    } 
    //}

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
 }
 // continue building the DriveCommand and follow the video of the zoom