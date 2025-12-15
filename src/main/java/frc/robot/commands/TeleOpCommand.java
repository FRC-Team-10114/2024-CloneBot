package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriverJoystick;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class TeleOpCommand extends Command {
    
    private final Drivetrain drivetrain;

    private final DriverJoystick driverJoystick;

    public TeleOpCommand(Drivetrain drivetrain, DriverJoystick driverJoystick) {

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
        double rot = MathUtil.applyDeadband(
            -driverJoystick.getRightX(), 
            0.05);

        drivetrain.teleOpDrive(xSpeed, ySpeed, rot);
    }
}
