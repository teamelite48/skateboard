package frc.robot.subsystems.drive.components;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.subsystems.drive.DriveConfig.*;
import static frc.robot.subsystems.drive.lib.SwerveMath.*;

public class AngleController {

    private final CANSparkMax motorController;
    private final SparkPIDController pidController;
    private final RelativeEncoder motorEncoder;
    private final CANcoder absoluteEncoder;

    private double targetAngle = 0.0;
    private int resetIteration = 0;

    public AngleController(int motorId, int absoluteEncoderId, double offsetDegrees) {

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        var config = new CANcoderConfiguration();
        
        config.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1); // TODO: need to update logic because we no longer have 0 to 360 output
        config.MagnetSensor.withMagnetOffset(-(offsetDegrees / 360));
        config.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        
        absoluteEncoder.getConfigurator().apply(config);

        
        motorController = new CANSparkMax(motorId, MotorType.kBrushless);

        motorController.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motorController.setSmartCurrentLimit((int) ANGLE_MOTOR_CURRENT_LIMIT);

        motorController.setInverted(ANGLE_MOTOR_INVERTED);

        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motorController.setIdleMode(CANSparkMax.IdleMode.kCoast);

        motorEncoder = motorController.getEncoder();
        motorEncoder.setPositionConversionFactor(ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR);
        motorEncoder.setVelocityConversionFactor(ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR / 60.0);
        motorEncoder.setPosition(getAbsoluteAngle());

        pidController = motorController.getPIDController();
        pidController.setP(1.0);
        pidController.setI(0.0);
        pidController.setD(0.1);
        pidController.setFeedbackDevice(motorEncoder);
    }

    public void setAngle(double desiredAngle) {

        double currentAngle = motorEncoder.getPosition();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                currentAngle = getAbsoluteAngle();
                motorEncoder.setPosition(currentAngle);

                resetIteration = 0;
            }
        }
        else {
            resetIteration = 0;
        }

        var currentAngleMod = normalizeAngle(currentAngle);

        // The target angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedDesiredAngle = desiredAngle + currentAngle - currentAngleMod;

        if (desiredAngle - currentAngleMod > PI) {
            adjustedDesiredAngle -= TAU;
        }
        else if (desiredAngle - currentAngleMod < -PI) {
            adjustedDesiredAngle += TAU;
        }

        this.targetAngle = adjustedDesiredAngle;

        pidController.setReference(this.targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return normalizeAngle(motorEncoder.getPosition());
    }

    public double getAbsoluteAngle() {
        
        return normalizeAngle(absoluteEncoder.getAbsolutePosition().getValue() * TAU);
    }
}