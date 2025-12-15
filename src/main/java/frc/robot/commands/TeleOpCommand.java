package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriverJoystick;
import frc.robot.subsystems.Drivetrain;

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
        double xSpeed = -driverJoystick.getLeftY();
        double ySpeed = driverJoystick.getLeftX();
        double rot = driverJoystick.getRightX();

        drivetrain.teleOpDrive(xSpeed, ySpeed, rot);
    }
}
