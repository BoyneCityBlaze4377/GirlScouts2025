package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralAffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroCoralWrist extends InstantCommand {
  private final CoralAffector m_affector;

  public ZeroCoralWrist(CoralAffector affector) {
    m_affector = affector;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_affector.resetWristEncoder();
  }
}
