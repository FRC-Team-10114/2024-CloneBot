// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class ModuleConstants {

    public static final double WilliamConstant = 1.042;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.95;
    public static final double kTurningMotorGearRatio = 1 / 19.6;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNeoFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class MotorConstants {
    public static final double kNeoFreeSpeedRpm = 6784;
  }

  public static final class DrivetrainConstants {
    // Max Speeds
    public static final double kMaxSpeedMetersPerSecond = 6.5;
    public static final double kTeleOpSpeedMetersPerSecond = 2;

    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * 1.8 * Math.PI;

    // ID's: { driveMotorID, steerMotorID, absEncoderID }
    public static final int[] kFrontLeftModuleIDs = { 4, 8, 12 };
    public static final int[] kFrontRightModuleIDs = { 3, 7, 11 };
    public static final int[] kBackLeftModuleIDs = { 2, 6, 10 };
    public static final int[] kBackRightModuleIDs = { 1, 5, 9 };

    // Inverted: { driveMotorInverted, steerMotorInverted }
    public static final boolean[] kFrontLeftModuleInverted = { false, true };
    public static final boolean[] kFrontRightModuleInverted = { true, true };
    public static final boolean[] kBackLeftModuleInverted = { false, true };
    public static final boolean[] kBackRightModuleInverted = { true, true };


    // Locations of the modules relative to the robot center
    public static final Translation2d[] moduleLocations = new Translation2d[] {
        new Translation2d(0.278, 0.278),
        new Translation2d(0.278, -0.278),
        new Translation2d(-0.278, 0.278),
        new Translation2d(-0.278, -0.278)
    };

    // Slew Rates
    public static final double kTeleOpDrivePositiveSlewRate = 5;
    public static final double kTeleOpDriveNegativeSlewRate = 20;
  }
}
