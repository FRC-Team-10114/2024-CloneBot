package frc.robot.subsystems.Dashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardHelper extends SubsystemBase {

    private Pose2d robotPose;
    private SendableChooser<Command> autoChooser;

    public DashboardHelper() {
        this.robotPose = Pose2d.kZero;
        this.autoChooser = AutoBuilder.buildAutoChooser();
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
        SmartDashboard.putNumberArray(
                "robotPose",
                new double[] {
                        robotPose.getX(),
                        robotPose.getY(),
                        robotPose.getRotation().getRadians()
                });

        SmartDashboard.putData("Auto Modes", autoChooser);

    }

    public void updatePose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

}
