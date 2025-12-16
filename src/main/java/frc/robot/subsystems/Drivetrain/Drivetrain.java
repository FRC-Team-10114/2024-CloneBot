package frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Dashboard.DashboardHelper;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final AHRS gyro;

    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    private final DashboardHelper dashboardHelper;

    private double MaxDriveSpeed = DrivetrainConstants.kTeleOpSpeedMetersPerSecond;

    public Drivetrain() {

        this.frontLeft = new SwerveModule(
                DrivetrainConstants.kFrontLeftModuleIDs[0],
                DrivetrainConstants.kFrontLeftModuleIDs[1],
                DrivetrainConstants.kFrontLeftModuleIDs[2],
                DrivetrainConstants.kFrontLeftModuleInverted[0],
                DrivetrainConstants.kFrontLeftModuleInverted[1]);
        this.frontRight = new SwerveModule(
                DrivetrainConstants.kFrontRightModuleIDs[0],
                DrivetrainConstants.kFrontRightModuleIDs[1],
                DrivetrainConstants.kFrontRightModuleIDs[2],
                DrivetrainConstants.kFrontRightModuleInverted[0],
                DrivetrainConstants.kFrontRightModuleInverted[1]);
        this.backLeft = new SwerveModule(
                DrivetrainConstants.kBackLeftModuleIDs[0],
                DrivetrainConstants.kBackLeftModuleIDs[1],
                DrivetrainConstants.kBackLeftModuleIDs[2],
                DrivetrainConstants.kBackLeftModuleInverted[0],
                DrivetrainConstants.kBackLeftModuleInverted[1]);
        this.backRight = new SwerveModule(
                DrivetrainConstants.kBackRightModuleIDs[0],
                DrivetrainConstants.kBackRightModuleIDs[1],
                DrivetrainConstants.kBackRightModuleIDs[2],
                DrivetrainConstants.kBackRightModuleInverted[0],
                DrivetrainConstants.kBackRightModuleInverted[1]);

        this.gyro = new AHRS(NavXComType.kMXP_SPI);

        this.kinematics = new SwerveDriveKinematics(
                DrivetrainConstants.moduleLocations);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                this.getRotation2d(),
                this.getModulePositions(),
                Pose2d.kZero);

        this.xLimiter = new SlewRateLimiter(DrivetrainConstants.kTeleOpDriveSlewRate);
        this.yLimiter = new SlewRateLimiter(DrivetrainConstants.kTeleOpDriveSlewRate);
        this.rotLimiter = new SlewRateLimiter(DrivetrainConstants.kTeleOpDriveSlewRate);

        this.dashboardHelper = new DashboardHelper();

        AutoBuilderConfigure();
    }

    // Basic Drivetrain Methods

    public double getHeading() {
        return -gyro.getAngle();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.kinematics.toChassisSpeeds(
                this.getModuleStates());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    public void drive(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds optimizedSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        var states = kinematics.toSwerveModuleStates(optimizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states, DrivetrainConstants.kMaxSpeedMetersPerSecond);

        this.frontLeft.setState(states[0]);
        this.frontRight.setState(states[1]);
        this.backLeft.setState(states[2]);
        this.backRight.setState(states[3]);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevMeters) {
        this.poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevMeters);
    }

    // Swerve Control Methods

    public void teleOpDrive(double Xinput, double Yinput, double Rotinput) {

        Xinput = this.xLimiter.calculate(Xinput);
        Yinput = this.yLimiter.calculate(Yinput);
        Rotinput = this.rotLimiter.calculate(Rotinput);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
                Xinput * MaxDriveSpeed,
                Yinput * MaxDriveSpeed,
                Rotinput * DrivetrainConstants.kMaxAngularSpeedRadiansPerSecond);

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                getRotation2d());

        this.drive(robotRelativeSpeeds);
    }

    public void fullSpeedMode() {
        this.MaxDriveSpeed = DrivetrainConstants.kMaxSpeedMetersPerSecond;

        this.xLimiter.reset(DrivetrainConstants.kMaxSlewRate);
        this.yLimiter.reset(DrivetrainConstants.kMaxSlewRate);
        this.rotLimiter.reset(DrivetrainConstants.kMaxSlewRate);
    }

    public void halfSpeedMode() {
        this.MaxDriveSpeed = DrivetrainConstants.kTeleOpSpeedMetersPerSecond;

        this.xLimiter.reset(DrivetrainConstants.kTeleOpDriveSlewRate);
        this.yLimiter.reset(DrivetrainConstants.kTeleOpDriveSlewRate);
        this.rotLimiter.reset(DrivetrainConstants.kTeleOpDriveSlewRate);
    }

    // For testing individual swerve modules

    public void swerveTest() {
        this.frontLeft.moduleTest();
        this.frontRight.moduleTest();
        this.backLeft.moduleTest();
        this.backRight.moduleTest();
    }

    // _____________AUTONOMOUS METHOD_____________

    public void AutoBuilderConfigure() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
                this.getRotation2d(),
                this.getModulePositions());

        dashboardHelper.updatePose(getPose());
    }

    // Commands

    public Command fullSpeedCommand() {
        return runOnce(() -> fullSpeedMode());
    }

    public Command halfSpeedCommand() {
        return runOnce(() -> halfSpeedMode());
    }
}