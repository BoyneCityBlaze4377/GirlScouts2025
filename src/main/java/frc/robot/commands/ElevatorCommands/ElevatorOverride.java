package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorOverride extends Command {
  private final Elevator m_elevator;
  private final Joystick m_stick;

  /** Creates a new ElevatorCommand. */
  public ElevatorOverride(Elevator elevator, Joystick stick) {
    m_elevator = elevator;
    m_stick = stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = -m_stick.getRawAxis(1) * ElevatorConstants.overrideSpeed;
    if (Math.abs(input) < .05) {
      m_elevator.lockElevator();
    } else {
      m_elevator.set(input);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
