package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllToSetPosition extends Command {
  private final Elevator m_elevator;
  private final CoralAffector m_coralAffector;
  private final double elevatorTarget, coralWristTarget;
  private final Position position;
  
  /** Creates a new SelectPosition. */
  public AllToSetPosition(Elevator elevator, CoralAffector coralAffector, Position desiredPosition) {
    m_elevator = elevator;
    m_coralAffector = coralAffector;
    position = desiredPosition;

    elevatorTarget = AutoAimConstants.positionValues.get(position)[0];
    coralWristTarget = AutoAimConstants.positionValues.get(position)[1];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_coralAffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralAffector.setSetpoint(coralWristTarget);
    m_coralAffector.setIsOverride(false);
    m_elevator.setSetpoint(elevatorTarget);

    m_elevator.setPositionString("Going to " + position.toString());
    m_elevator.setCurrentPosition(position);
    m_elevator.setAtPos(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.PIDMove();
    m_coralAffector.PIDMoveWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
    m_coralAffector.lockWrist();

    m_elevator.setPositionString("At " + position.toString());
    m_elevator.setAtPos(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralAffector.atSetpoint() && m_elevator.atSetpoint();
  }
}
