package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverJoystick extends XboxController {
    
    public DriverJoystick(int port) {
        super(port);
    }

    public Trigger swerveTest() {
        return new Trigger(this::getAButton);
    }
}
