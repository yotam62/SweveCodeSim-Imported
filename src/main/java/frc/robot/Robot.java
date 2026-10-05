// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;

import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static ArrayList<String> DisconnectedMotorNames = new ArrayList<String>();

  private final RobotContainer m_robotContainer;
    
  //these decide whether we will disable a limelight
  public static boolean useLimelightFour = true;
  public static boolean useLimelightThreeGS = true;
  public static boolean useLimelightThreeGF = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

         String fullList_disconnections = "";
        for (String motorName : DisconnectedMotorNames) {
            fullList_disconnections += motorName + ", " + "\n";
        }
        
        SmartDashboard.putString("Disconnected Motors", fullList_disconnections);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}

    public static void reportDisconnection(String motorName) {
    DisconnectedMotorNames.add(motorName);
  }

  public static void removeDisconnection(String motorName) {
    DisconnectedMotorNames.remove(motorName);
  }

}
