package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * 
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class TurretTargetCommandGroup extends SequentialCommandGroup {
    /**
     * Creates a new ComplexAuto.
     *
     * @param drive The drive subsystem this command will run on
     * @param hatch The hatch subsystem this command will run on
     */
    public TurretTargetCommandGroup(LimelightSubsystem limelight, TurretSubsystem turret) {
      addCommands(
          // Drive forward the specified distance
          new TurretFindTargetCommand(turret, limelight),
  
  
          // Drive backward the specified distance
          new TurretLockOnCommand(turret, limelight));
    }
  
  }