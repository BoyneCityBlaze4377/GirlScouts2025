package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPosition extends Command {
  private final Elevator m_elevator;
  private final double desiredPos;
  private final PIDController elevatorController;

  /** Creates a new ElevatorToPosition. */
  public ElevatorToPosition(Elevator elevator, Position DesiredPos) {
    m_elevator = elevator;
    desiredPos = AutoAimConstants.positionValues.get(DesiredPos)[0];
    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.kTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(elevatorController.calculate(m_elevator.getEncoderVal(), desiredPos), 
                                   ElevatorConstants.maxDownSpeed, ElevatorConstants.maxUpSpeed);
    m_elevator.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorController.atSetpoint();
  }
}
