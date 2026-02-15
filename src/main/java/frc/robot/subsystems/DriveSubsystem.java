package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // --- “Real” objects (these also work in sim) ---
  private final MotorController leftMotor = new PWMSparkMax(Constants.Drive.LEFT_MOTOR_PWM);
  private final MotorController rightMotor = new PWMSparkMax(Constants.Drive.RIGHT_MOTOR_PWM);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private final Encoder leftEncoder = new Encoder(0, 1);
  private final Encoder rightEncoder = new Encoder(2, 3);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();

  // --- Simulation-only objects ---
  private DifferentialDrivetrainSim drivetrainSim;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private ADXRS450_GyroSim gyroSim;

  private double lastSimTime = Timer.getFPGATimestamp();

  public DriveSubsystem() {
    // Typical inversion for a simple 2-motor drive:
    rightMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_PULSE);

    gyro.reset();
    resetEncoders();

  odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    leftEncoder.getDistance(),
    rightEncoder.getDistance());

    SmartDashboard.putData("Field", field);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    gyro.reset();
    odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    field.setRobotPose(pose);

    if (RobotBase.isSimulation() && drivetrainSim != null) {
      drivetrainSim.setPose(pose);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive/LeftOutput", leftMotor.get());
    SmartDashboard.putNumber("Drive/RightOutput", rightMotor.get());

    // Update odometry from sensors (real OR sim-fed sensors)
    odometry.update(
        gyro.getRotation2d(),
        leftEncoder.getDistance(),
        rightEncoder.getDistance());

    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Drive/PoseX", getPose().getX());
    SmartDashboard.putNumber("Drive/PoseY", getPose().getY());
    SmartDashboard.putNumber("Drive/HeadingDeg", gyro.getAngle());
  }

  private void setupSimulation() {
    // A reasonable “starter” drivetrain sim:
    // - 2 CIM motors per side
    // - gear ratio: ~10.71
    // - track width: from constants
    // - wheel radius: from constants
    // - mass and MOI are approximations
    drivetrainSim =
        new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(1.0, 3.0, 1.0, 0.5),
      DCMotor.getCIM(2),
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.TRACK_WIDTH_METERS,
      Constants.Drive.WHEEL_RADIUS_METERS,
      VecBuilder.fill(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01));

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    gyroSim = new ADXRS450_GyroSim(gyro);
  }

  @Override
  public void simulationPeriodic() {
    if (drivetrainSim == null) return;

    double now = Timer.getFPGATimestamp();
    double dt = now - lastSimTime;
    lastSimTime = now;

    // Motor output is -1..1; convert to volts (assume 12V battery)
    double leftVolts = leftMotor.get() * 12.0;
    double rightVolts = rightMotor.get() * 12.0;

    drivetrainSim.setInputs(leftVolts, rightVolts);
    drivetrainSim.update(dt);

    // Update simulated encoders (meters and meters/sec)
    leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());

    // Update simulated gyro (degrees)
    gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees()); // sign often needs flipping
  }
}
