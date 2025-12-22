package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotState {

    public Optional<DriverStation.Alliance> alliance;

    public Pose2d robotPose;

    public RobotState() {
        this.alliance = DriverStation.getAlliance();
        this.robotPose = Pose2d.kZero;
    }

    public void updatePose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }
}
