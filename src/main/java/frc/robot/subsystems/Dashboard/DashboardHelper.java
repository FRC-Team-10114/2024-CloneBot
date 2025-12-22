package frc.robot.subsystems.Dashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class DashboardHelper extends SubsystemBase {

    private RobotState state;

    private SendableChooser<Command> autoChooser;

    public DashboardHelper(RobotState state) {
        this.autoChooser = AutoBuilder.buildAutoChooser();
        this.state = state;
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
        SmartDashboard.putNumberArray(
                "robotPose",
                new double[] {
                        state.robotPose.getX(),
                        state.robotPose.getY(),
                        state.robotPose.getRotation().getRadians()
                });

        SmartDashboard.putData("Auto Modes", autoChooser);

    }
    public Command getAuto() {
        return autoChooser.getSelected();
    }

}
