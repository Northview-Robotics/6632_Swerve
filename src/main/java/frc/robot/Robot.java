// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotContainer;
//import robot subsystems
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.operatorinterface;
// import frc.robot.subsystems.LEDS;
// import frc.robot.commands.RobotContainer;

public class Robot extends TimedRobot {
  public Drive drive;
  public operatorinterface oi;
  public RobotContainer robotContainer;
  // public LEDS led;
  public Command getAutonomousCommand;

  public Robot() {
  }

  @Override
  public void robotInit() {
    drive = Drive.getInstance();
    oi = operatorinterface.getInstance();
    robotContainer = new RobotContainer();
    //led = LEDS.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    getAutonomousCommand = robotContainer.getAutonomousCommand();
    
    if (getAutonomousCommand != null){
      getAutonomousCommand.schedule();
   }
  }

  @Override
  public void autonomousPeriodic() {
    // led.AutonWave();
  }

  @Override
  public void teleopInit() {
    if (getAutonomousCommand != null){
      getAutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // led.Wave();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
