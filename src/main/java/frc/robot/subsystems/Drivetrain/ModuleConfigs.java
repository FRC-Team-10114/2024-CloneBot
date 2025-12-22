package frc.robot.subsystems.Drivetrain;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public class ModuleConfigs {
    public static final SparkFlexConfig driveConfig = new SparkFlexConfig();
    public static final SparkMaxConfig steerConfig = new SparkMaxConfig();

    static {
        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = ModuleConstants.kTurningMotorGearRatio * 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        driveConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.13, 0.000, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);
                
        steerConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        steerConfig.encoder
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
        steerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.65, 0.0007, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI);
    }
}