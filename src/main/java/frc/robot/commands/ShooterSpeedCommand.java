// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShooterSpeedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem m_LimelightSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterSpeedCommand(LimelightSubsystem limelightsubsystem) {
    m_LimelightSubsystem = limelightsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double x = m_LimelightSubsystem.getAprilTagDistanceRobotSpace().getX();
    // double y =Math.abs(m_LimelightSubsystem.getAprilTagDistanceRobotSpace().getY()); 

    double d = (m_LimelightSubsystem.getTargetDistance() / 10);

    SmartDashboard.putNumber("Shooting Power", d);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
