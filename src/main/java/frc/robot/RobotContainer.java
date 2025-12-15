// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleOpCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final DriverJoystick driverJoystick = new DriverJoystick(0);
  private final Drivetrain drivetrain = new Drivetrain();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    this.drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverJoystick));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    this.driverJoystick.swerveTest().whileTrue(new TestCommand(drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
