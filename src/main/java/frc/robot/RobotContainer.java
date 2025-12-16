// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleOpCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.Controller.DriverJoystick;
import frc.robot.subsystems.Dashboard.DashboardHelper;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {

  private final DriverJoystick driverJoystick = new DriverJoystick(0);
  private final Drivetrain drivetrain = new Drivetrain();
  private final DashboardHelper dashboardHelper = new DashboardHelper();

  public RobotContainer() {

    this.drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverJoystick));

    configureBindings();
  }

  private void configureBindings() {

    this.driverJoystick.swerveTest()
        .whileTrue(new TestCommand(drivetrain));
    this.driverJoystick.zeroHeading()
        .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
    this.driverJoystick.fullSpeedMode()
        .onTrue(drivetrain.fullSpeedCommand())
        .onFalse(drivetrain.halfSpeedCommand());
  }

  public Command getAutonomousCommand() {
    return dashboardHelper.getAuto();
  }
}
