package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    
    private final SparkFlex driveMotor;
    private final SparkMax steerMotor;
    
    private final RelativeEncoder driveEncoder, steerEncoder;

    private final SparkClosedLoopController driveController, steerController;

    private final CANcoder absEncoder;

    public SwerveModule(
        int driveMotorID,
        int steerMotorID,
        int absEncoderID,
        boolean isDriveMotorInverted,
        boolean isSteerMotorInverted
    ) {
        this.driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
        this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);

        this.driveController = this.driveMotor.getClosedLoopController();
        this.steerController = this.steerMotor.getClosedLoopController();

        this.driveEncoder = this.driveMotor.getEncoder();
        this.steerEncoder = this.steerMotor.getEncoder();

        this.absEncoder = new CANcoder(absEncoderID);

        configure(isDriveMotorInverted, isSteerMotorInverted);

        resetEncoders();
    }

    public void configure(boolean isDriveMotorInverted, boolean isSteerMotorInverted) {
        this.driveMotor.configure(
            Configs.driveConfig.inverted(isDriveMotorInverted), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        this.steerMotor.configure(
            Configs.steerConfig.inverted(isSteerMotorInverted), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    public double getCANcoderRad() {
        double angle = this.absEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        SmartDashboard.putNumber("absolutEncoderAngle", angle);
        return angle;
    }

    public void resetEncoders() {
        this.driveEncoder.setPosition(0);
        this.steerEncoder.setPosition(getCANcoderRad());
    }

    public double getDrivePosition() {
        return this.driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return this.steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    public double getSteerVelocity() {
        return this.steerEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.getDrivePosition(),
            Rotation2d.fromRadians(this.getSteerPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.getDriveVelocity(),
            Rotation2d.fromRadians(this.getSteerPosition())
        );
    }

    public void setState(SwerveModuleState state) {
        state.optimize(
            Rotation2d.fromRadians(this.getSteerPosition()));

        this.driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        this.steerController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    //_____________SWERVE MODULE TEST METHODS_____________

    public void moduleTest() {
        this.steerController.setReference(0, ControlType.kPosition);
    }
}
