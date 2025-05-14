package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAffector;

public class CoralCollect extends Command {
  private final CoralAffector m_affector;

  /** Creates a new CoralCollect. */
  public CoralCollect(CoralAffector coralAffector) {
    m_affector = coralAffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(//m_affector
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_affector.collect();
    //if (!m_affector.hasCoral()) m_affector.collect();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_affector.stopAffector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_affector.hasCoral();
  }
}
