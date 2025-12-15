// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.TeleOpCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {

  private final DriverJoystick driverJoystick = new DriverJoystick(0);
  private final Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {

    this.drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverJoystick));

    configureBindings();
  }

  private void configureBindings() {
    this.driverJoystick.swerveTest().whileTrue(new TestCommand(drivetrain));
  }

  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto("New Auto") ;
  }
}
