package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/** A command that will turn the robot to the specified angle. */
public class TurretLockOnCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurretLockOnCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
    super(
        new PIDController(0.01, 0, 0),
        // Close loop on heading
        limelight::getTargetRotation,
        // Set reference to target
        0,
        // Pipe output to turn robot
        output -> turret.runTurretSpeed(-output),
        // Require the drive
        limelight, turret);

    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(1, 1);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}