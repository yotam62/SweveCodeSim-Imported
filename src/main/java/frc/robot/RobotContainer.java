package frc.robot;

import frc.robot.lib.BLine.*;

import java.util.List;
import java.util.Random;
import java.util.Set;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.Alliance;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.GamepadButton;
import frc.robot.drive.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.ArmSim;
import frc.robot.TwoJointArm.ArmSubsystem;
import frc.robot.TwoJointArm.TrapezoidalTrajGenerator;
import frc.robot.TwoJointArm.ArmCommands.StayCommand;
import frc.robot.TwoJointArm.ArmCommands.TrajectoryFollower;

public class RobotContainer {
  public final Drive drive;
  private FollowPath.Builder pathBuilder; //  not final, initialized in constructor
  Random random = new Random();
  TrapezoidalTrajGenerator trajGenerator = new TrapezoidalTrajGenerator();
  ArmSubsystem arm = new ArmSubsystem(new ArmSim());

  CommandGamepad controller = new CommandGamepad(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(
        new ModuleReal(SwerveConstants.Mod0.constants, 0),  //fl
        new ModuleReal(SwerveConstants.Mod1.constants, 1),  //bl
        new ModuleReal(SwerveConstants.Mod2.constants, 2),  //br
        new ModuleReal(SwerveConstants.Mod3.constants, 3)); //fr

    }

    else {
      
          drive = new Drive(
      new ModuleSim(0, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(1, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(2, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(3, SwerveConstants.SwerveModuleConstants));

    } 
    configureBindings();
  }


   private void configureBindings() {
    drive.setDefaultCommand(
      new DriveCommand(
        () -> -controller.getLeftX(),
        () -> -controller.getLeftY(),
        () -> -controller.getRightX(),
        drive,
        controller
      )); 

      controller.faceLeft().onTrue(FeedforwardCharacterization.feedforwardCommand(drive, controller.getHID()));
      controller.button(8).onTrue(
        Commands.runOnce(() -> drive.setPose(
          new Pose2d(
            drive.SwervePoseEstimator.getEstimatedPosition().getTranslation(), 
            MatchState.getAlliance().get().equals(Alliance.BLUE) ? Rotation2d.ZERO : Rotation2d.fromDegrees(180))), 
            drive)); 


            controller.a().onTrue(
    Commands.defer(
        () -> new TrajectoryFollower(
            trajGenerator.createTraj(arm.getcurrentState(), arm.createRandomArmPointTraj(), List.of()),
            arm
        ),
        Set.of(arm)));

        arm.setDefaultCommand(new StayCommand(arm));


    controller.b().onTrue(
    Commands.defer(
        () -> new TrajectoryFollower(
            trajGenerator.createTraj(arm.getcurrentState(), arm.createRandomArmPointTraj(), List.of()),
            arm
        ),
        Set.of(arm)));


 }

  //   pathBuilder = new FollowPath.Builder(
  //       drive,
  //       drive::getPose,
  //       drive::getChassisVelocities,
  //       drive::runVelocity,
  //       new PIDController(5.0, 0.0, 0.0),
  //       new PIDController(3.5, 0.0, 0.0),
  //       new PIDController(0.0, 0.0, 0.0)
  //   )
  //   .withDefaultShouldFlip();    


  //   configureBindings();
  // }

  // private void configureBindings() {
  //   drive.setDefaultCommand(
  //     new DriveCommand(
  //       () -> -controller.getLeftX(),
  //       () -> -controller.getLeftY(),
  //       () -> -controller.getHID().getRawAxis(2),
  //       drive,
  //       controller
  //     ));

  //   controller.x().whileTrue(new AutoAlignHubCommand(drive, 
  //       () -> -controller.getLeftX(), 
  //       () -> -controller.getLeftY(), 
  //       () -> -controller.getHID().getRawAxis(2), 
  //       controller));

  //   controller.x().whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("X pressed", true)))
  //       .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("X pressed", false)));
  // }


  // public Command getAutonomousCommand() {

  //       boolean isAllianceRed = MatchState.getAlliance().isPresent() 
  //       && MatchState.getAlliance().get() == Alliance.RED;

    
  //   Path pathA = new Path("practice_pathA");
  //   Path pathB = new Path("practice_pathB");
  //   Path tuningPID = new Path("tuning PID");


  //   Pose2d pathBSStart = isAllianceRed? flipPose(pathB.getStartPose()) : pathB.getStartPose();
  //   Pose2d pathASS = isAllianceRed? flipPose(pathA.getStartPose()) : pathA.getStartPose();




  //   return Commands.sequence(

  //       new InstantCommand(() -> drive.resetPosition(pathASS)), // only reset once at start
  //       //pathBuilder.build(tuningPID)
  //       pathBuilder.build(pathA),
  //       new ShootingPathsAuto(drive, pathBSStart.getTranslation()),
  //       pathBuilder.build(pathB)
  //   );
  // }
  
  //   private Pose2d flipPose(Pose2d pose) {
    
  //     return new Pose2d(
  //       16.541 - pose.getX(),
  //       8.21 -pose.getY(),
  //       pose.getRotation().times(-1));
  //   }
  public Command getAutonomousCommand() {
    return Commands.none();
  }

}