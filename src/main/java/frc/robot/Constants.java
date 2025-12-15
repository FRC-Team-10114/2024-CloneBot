// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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

    public static final double kTrackWidthMeters = Units.inchesToMeters(21.73);
    public static final double kWheelBaseMeters = Units.inchesToMeters(21.73);

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(1080.0);

    public static final int[] kFrontLeftModuleIDs = { 1, 5, 9 };
    public static final int[] kFrontRightModuleIDs = { 2, 6, 10 };
    public static final int[] kBackLeftModuleIDs = { 3, 7, 11 };
    public static final int[] kBackRightModuleIDs = { 4, 8, 12 };

    public static final boolean[] kFrontLeftModuleInverted = {false, false};
    public static final boolean[] kFrontRightModuleInverted = {false, false};
    public static final boolean[] kBackLeftModuleInverted = {false, false};
    public static final boolean[] kBackRightModuleInverted = {false, false};

    public static final Translation2d[] moduleLocations = new Translation2d[] {
        new Translation2d(0.278, 0.278),
        new Translation2d(0.278, -0.278),
        new Translation2d(-0.278, 0.278),
        new Translation2d(-0.278, -0.278)
    };
    public static double kTeleOpDriveSlewRate = 7.0;
  }
}
