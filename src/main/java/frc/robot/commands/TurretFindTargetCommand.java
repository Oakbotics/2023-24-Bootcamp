// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.TurretFindTargetCommand;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurretFindTargetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem m_TurretSubsystem;
  private final LimelightSubsystem m_LimelightSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretFindTargetCommand(TurretSubsystem Turret, LimelightSubsystem Limelight) {
    m_TurretSubsystem = Turret;
    m_LimelightSubsystem = Limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(m_LimelightSubsystem.getPoseDiff()+m_LimelightSubsystem.getTargetDegree())>Math.abs(m_LimelightSubsystem.getPoseDiff()-m_LimelightSubsystem.getTargetDegree())){
        m_TurretSubsystem.runTurretSpeed(1);
    } 
    else if(Math.abs(m_LimelightSubsystem.getPoseDiff()+m_LimelightSubsystem.getTargetDegree())<Math.abs(m_LimelightSubsystem.getPoseDiff()-m_LimelightSubsystem.getTargetDegree())){
        m_TurretSubsystem.runTurretSpeed(-1);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretSubsystem.runTurretSpeed(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LimelightSubsystem.getTagWithinSight();
  }
}
