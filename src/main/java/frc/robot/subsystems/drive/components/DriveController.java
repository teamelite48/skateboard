package frc.robot.subsystems.drive.components;

import static frc.robot.subsystems.drive.DriveConfig.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class DriveController {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private double targetVelocity = 0.0;

    public DriveController(int id) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.setInverted(DRIVE_MOTOR_INVERTED);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) DRIVE_MOTOR_CURRENT_LIMIT);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        encoder = motor.getEncoder();

        this.encoder.setPositionConversionFactor(DRIVE_POSITION_TO_METERS_CONVERSION_FACTOR);
        this.encoder.setVelocityConversionFactor(DRIVE_POSITION_TO_METERS_CONVERSION_FACTOR / 60.0);
    }

    public void setVelocity(double metersPerSecond) {
        this.targetVelocity = metersPerSecond;
        motor.setVoltage(this.targetVelocity / MAX_METERS_PER_SECOND * NOMINAL_VOLTAGE);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return encoder.getVelocity();
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }
}


