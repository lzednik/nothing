package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends Command {
  private final DriveSubsystem drive;
  private final DoubleSupplier fwd;
  private final DoubleSupplier rot;

  public TeleopDrive(DriveSubsystem drive, DoubleSupplier fwd, DoubleSupplier rot) {
    this.drive = drive;
    this.fwd = fwd;
    this.rot = rot;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double fwdValue = applyDeadband(fwd.getAsDouble());
    double rotValue = applyDeadband(rot.getAsDouble());

    SmartDashboard.putNumber("Drive/TeleopFwd", fwdValue);
    SmartDashboard.putNumber("Drive/TeleopRot", rotValue);

    drive.arcadeDrive(fwdValue, rotValue);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyDeadband(double x) {
    return Math.abs(x) < 0.08 ? 0.0 : x;
  }
}
