// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoPicker.AutoRoutines;
import frc.robot.SubSystem.Logging.NerdLog;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  int routine;

  public Robot() {
    //possible fix if photonvision Web GUI doesn't load during competition
    PortForwarder.add(5800, "photonvision.local", 5800);
     m_robotContainer = new RobotContainer();
     routine = 0;
     NerdLog.logDouble("Auto Routine", 0);

     
  }

  @Override
  public void robotPeriodic() {
    //request Auto:
    routine = (int) (NerdLog.getdouble("Auto Routine") + 0.5);

    m_robotContainer.roboPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    //the auto is chosen the moment autonomus starts
    switch (routine) {
      case 1: AutoPicker.pickAuto(AutoRoutines.SHOOT_BALLS); break;
      case 2: AutoPicker.pickAuto(AutoRoutines.SHOOT_BALLS_AND_CLIMB);
      case 3: AutoPicker.pickAuto(AutoRoutines.LEFT_OVER_BUMP); break;
      case 4: AutoPicker.pickAuto(AutoRoutines.RIGHT_OVER_BUMP); break;
      default: AutoPicker.pickAuto(AutoRoutines.CRY); break;
     }


    //AutoPicker.pickAuto(AutoRoutines.SHOOT_BALLS_AND_CLIMB); // the only auto we actually have
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
    m_robotContainer.enabled();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {}
}
