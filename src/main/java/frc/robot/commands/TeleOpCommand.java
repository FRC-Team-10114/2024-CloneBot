package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriverJoystick;
import frc.robot.subsystems.Drivetrain;

public class TeleOpCommand extends Command {
    
    private final Drivetrain drivetrain;

    private final DriverJoystick driverJoystick;

    private final PIDController headingController;

    public TeleOpCommand(Drivetrain drivetrain, DriverJoystick driverJoystick) {
        this.headingController = new PIDController(0.03, 0, 0);
        this.headingController.enableContinuousInput(-180, 180);
        this.drivetrain = drivetrain;
        this.driverJoystick = driverJoystick;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(
            -driverJoystick.getLeftY(), 
            0.05);
        double ySpeed = MathUtil.applyDeadband(
            -driverJoystick.getLeftX(), 
            0.05);
        double turn = MathUtil.applyDeadband(
            -driverJoystick.getRightX(), 
            0.05);

        double rot = this.headingController.calculate(this.drivetrain.getHeading(), 0);

        drivetrain.teleOpDrive(xSpeed, ySpeed, rot);
    }
}
