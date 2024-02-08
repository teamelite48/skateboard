// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.components.SwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class DriveSubsystem extends SubsystemBase{

    enum Gear {
        Low,
        High
    }

    private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SLEW_RATE);

    private Gear gear = Gear.Low;

    private final SwerveModule frontLeft = new SwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_ID,
        FRONT_LEFT_ANGLE_MOTOR_ID,
        FRONT_LEFT_ANGLE_ENCODER_ID,
        FRONT_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule frontRight = new SwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_ENCODER_ID,
        FRONT_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backLeft = new SwerveModule(
        BACK_LEFT_DRIVE_MOTOR_ID,
        BACK_LEFT_ANGLE_MOTOR_ID,
        BACK_LEFT_ANGLE_ENCODER_ID,
        BACK_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backRight = new SwerveModule(
        BACK_RIGHT_DRIVE_MOTOR_ID,
        BACK_RIGHT_ANGLE_MOTOR_ID,
        BACK_RIGHT_ANGLE_ENCODER_ID,
        BACK_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
    );

    private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.getYaw().getValue()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    public DriveSubsystem() {
        zeroGyro();
        configAutobuilder();
        initShuffleBoard();
    }

    private void configAutobuilder() {
        AutoBuilder.configureHolonomic(
            () -> odometry.getPoseMeters(),
            this::resetOdometry,
            this::getChassisSpeeds,
            this::setSwerveModuleStates,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                4.5,
                0.4,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                
                return false;
            },
            this
        );
    }

    public void periodic() {
        updateOdometry();
    }

    public void manualDrive(double x, double y, double rotation) {

        double speedModifier = MAX_OUTPUT;

        if (gear == Gear.Low) {
            speedModifier = LOW_GEAR_SPEED;
        }

        var chassisSpeeds = calculateChassisSpeeds(x * speedModifier, y * speedModifier, rotation * speedModifier);
        
        setSwerveModuleStates(chassisSpeeds);
    }
    
    public void setLowGear(){
        gear = Gear.Low;
    }

    public void setHighGear(){
        gear = Gear.High;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    // public void reverseGyro() {
    //     gyro.addYaw(180.0);
    // }

    public double getPitch() {
        return gyro.getRoll().getValue();
    }

    private ChassisSpeeds calculateChassisSpeeds(double x, double y, double rotation) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            -yLimiter.calculate(y) * MAX_METERS_PER_SECOND,
            -xLimiter.calculate(x) * MAX_METERS_PER_SECOND,
            -rotationLimiter.calculate(rotation) * MAX_ANGULAR_METERS_PER_SECOND,
            Rotation2d.fromDegrees(gyro.getYaw ().getValue())
        );
    }

    private SwerveModuleState[] calculateSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_METERS_PER_SECOND);

        return swerveModuleStates;
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        
        var states = calculateSwerveModuleStates(chassisSpeeds);
        
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }


    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    private void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw().getValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    private void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw().getValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    private void initShuffleBoard() {

        var driveTab = Shuffleboard.getTab("Drive");

        driveTab.addDouble("Pitch", () -> getPitch())
            .withPosition(0, 0);

            driveTab.addDouble("Yaw", () -> gyro.getYaw().getValue())
            .withPosition(1, 0);

        driveTab.addString("Odometry", () -> odometry.getPoseMeters().getTranslation().toString())
            .withPosition(2, 0)
            .withSize(2, 1);

        driveTab.addDouble("X Acceleration", () -> gyro.getAccelerationY().getValueAsDouble())
            .withPosition(3, 0);
            
        driveTab.addDouble("Y Acceleration", () -> gyro.getAccelerationX().getValueAsDouble())
            .withPosition(3, 1);

        driveTab.addString("Chassis Speeds", () -> getChassisSpeeds().toString())
            .withPosition(4,0)
            .withSize(2, 1);
    }
}