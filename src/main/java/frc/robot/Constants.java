package frc.robot;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    private OperatorConstants() {}
  }

  public static final class Drive {
    // PWM ports for simple sim drivetrain (easy to start)
    public static final int LEFT_MOTOR_PWM = 0;
    public static final int RIGHT_MOTOR_PWM = 1;

    // Basic drivetrain physical constants (reasonable defaults)
    public static final double TRACK_WIDTH_METERS = 0.6;     // distance between left/right wheels
    public static final double WHEEL_RADIUS_METERS = 0.0762; // 3" radius (6" diameter)

    // Encoder settings (simulation uses these)
    public static final int ENCODER_CPR = 1024;
    public static final double GEAR_RATIO = 10.71; // common-ish; not critical for sim demo

    // Derived: distance per encoder pulse (meters)
    public static final double DISTANCE_PER_PULSE =
        2.0 * Math.PI * WHEEL_RADIUS_METERS / (ENCODER_CPR * GEAR_RATIO);
  }

  private Constants() {}
}
