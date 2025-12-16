package frc.robot.subsystems.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverJoystick extends XboxController {
    
    public DriverJoystick(int port) {
        super(port);
    }

    public Trigger swerveTest() {
        return new Trigger(this::getAButton);
    }

    public Trigger zeroHeading() {
        return new Trigger(this::getBButton);
    }

    public Trigger fullSpeedMode() {
        return new Trigger(this::getRightBumperButton);
    }
}
