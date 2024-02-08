// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.subsystems.drive.DriveConfig.*;
import static frc.robot.subsystems.drive.lib.SwerveMath.*;

public class SwerveModule {

    private final DriveController driveController;
    private final AngleController angleController;

    public SwerveModule(
        int driveMotorId,
        int angleMotorId,
        int absoluteEncoderId,
        double angleOffsetDegrees
    ) {
        driveController = new DriveController(driveMotorId);
        angleController = new AngleController(angleMotorId, absoluteEncoderId, angleOffsetDegrees);

        initDashboard(driveMotorId);
    }

    public void setState(SwerveModuleState state) {

        double desiredAngle = normalizeAngle(state.angle.getRadians());
        double currentAngle = angleController.getCurrentAngle();
        double angleDifference = desiredAngle - currentAngle;

        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (angleDifference >= PI) {
            desiredAngle -= TAU;
        } else if (angleDifference < -PI) {
            desiredAngle += TAU;
        }

        angleDifference = desiredAngle - currentAngle; // Recalculate difference

        double desiredVelocity = state.speedMetersPerSecond;

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (angleDifference > PI / 2.0 || angleDifference < -PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            desiredAngle += PI;
            desiredVelocity *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        desiredAngle = normalizeAngle(desiredAngle);

        driveController.setVelocity(desiredVelocity);
        angleController.setAngle(desiredAngle);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveController.getCurrentPosition(),
            new Rotation2d(angleController.getCurrentAngle())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveController.getCurrentVelocity(),
            Rotation2d.fromRadians(angleController.getCurrentAngle())
        );
    }

    private void initDashboard(int driveMotorId) {

        var title = "undefined";

        switch (driveMotorId) {
            case FRONT_LEFT_DRIVE_MOTOR_ID:
                title = "Front Left";
                break;
            case FRONT_RIGHT_DRIVE_MOTOR_ID:
                title = "Front Right";
                break;
            case BACK_LEFT_DRIVE_MOTOR_ID:
                title = "Back Left";
                break;
            case BACK_RIGHT_DRIVE_MOTOR_ID:
                title = "Back Right";
                break;
        }

        var tab = Shuffleboard.getTab("Swerve Modules");
        var layout = tab.getLayout(title, BuiltInLayouts.kList);

        layout.addDouble("Target Velocity", () -> driveController.getTargetVelocity());
        layout.addDouble("Current Velocity", () -> driveController.getCurrentVelocity());

        layout.addDouble("Target Angle", () -> Math.toDegrees(angleController.getTargetAngle()));
        layout.addDouble("Current Angle", () -> Math.toDegrees(angleController.getCurrentAngle()));

        layout.addDouble("Absolute Angle", () -> Math.toDegrees(angleController.getAbsoluteAngle()));

        layout.addDouble("Position", () -> driveController.getCurrentPosition());
    }
}