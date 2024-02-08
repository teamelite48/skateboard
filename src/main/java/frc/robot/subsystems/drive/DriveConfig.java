package frc.robot.subsystems.drive;

public class DriveConfig {

    public static final int GYRO_ID = 5;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 21;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 22;
    public static final int FRONT_LEFT_ANGLE_ENCODER_ID = 31;
    public static final double FRONT_LEFT_ANGLE_OFFSET_DEGREES = 311.924 + 180.0;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 23;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 24;
    public static final int FRONT_RIGHT_ANGLE_ENCODER_ID = 33;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_DEGREES = 246.534;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 25;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 26;
    public static final int BACK_LEFT_ANGLE_ENCODER_ID = 35;
    public static final double BACK_LEFT_ANGLE_OFFSET_DEGREES = 179.648 + 180.0;

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 27;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 28;
    public static final int BACK_RIGHT_ANGLE_ENCODER_ID = 37;
    public static final double BACK_RIGHT_ANGLE_OFFSET_DEGREES = 9.1406;

    public static final double MAX_OUTPUT = 0.25;
    public static final double LOW_GEAR_SPEED = 0.25;
    public static final double SLEW_RATE = 2.0;

    public static final double WHEEL_DIAMETER = 0.10033;
    public static final double TRACKWIDTH_METERS = 0.66675;
    public static final double WHEELBASE_METERS = 0.57785;

    public static final double MAX_METERS_PER_SECOND = 5.0597;
    public static final double MAX_ANGULAR_METERS_PER_SECOND = MAX_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double DRIVE_MOTOR_CURRENT_LIMIT = 40.0;
    public static final double ANGLE_MOTOR_CURRENT_LIMIT = 30.0;

    public static final boolean DRIVE_MOTOR_INVERTED = true;
    public static final boolean ANGLE_MOTOR_INVERTED = true;

    public static final double DRIVE_MOTOR_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double ANGLE_MOTOR_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    public final static double DRIVE_POSITION_TO_METERS_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_MOTOR_REDUCTION;
    public final static double ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR = 2.0 * Math.PI * ANGLE_MOTOR_REDUCTION;

    public static final int ENCODER_RESET_ITERATIONS = 500;
    public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
}
