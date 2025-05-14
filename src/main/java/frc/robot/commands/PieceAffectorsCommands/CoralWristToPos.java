package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.CoralAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralWristToPos extends Command {
  private final CoralAffector m_affector;
  private final double desiredPos;

  /** Creates a new ElevatorToPosition. */
  public CoralWristToPos(CoralAffector affector, Position DesiredPos) {
    m_affector = affector;
    desiredPos = AutoAimConstants.positionValues.get(DesiredPos)[0];
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_affector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_affector.setSetpoint(desiredPos);
    m_affector.setIsOverride(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_affector.PIDMoveWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_affector.lockWrist(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_affector.atSetpoint();
  }
}
