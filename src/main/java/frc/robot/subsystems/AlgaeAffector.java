package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;

public class AlgaeAffector extends SubsystemBase {
  private final SparkMax algaeCollectorOne, algaeCollectorTwo;
  private final SparkMaxConfig algaeCollectorOneConfig, algaeCollectorTwoConfig;
  private final DigitalInput algaeDetector;
  private final GenericEntry hasAlgaeSender;

  /** Creates a new PieceAffector. */
  public AlgaeAffector() {
    /** Motors */
    algaeCollectorOne = new SparkMax(AffectorConstants.algaeCollectorOneID, MotorType.kBrushless);
    algaeCollectorTwo = new SparkMax(AffectorConstants.algaeCollectorTwoID, MotorType.kBrushless);

    algaeCollectorOneConfig = new SparkMaxConfig();
    algaeCollectorTwoConfig = new SparkMaxConfig();

    configMotorControllerDefaults();

    /** Beam Break */
    algaeDetector = new DigitalInput(SensorConstants.algaeBreakID);

    /** DashBoard Initialization */
    hasAlgaeSender = IOConstants.TeleopTab.add("HasAlgae", hasAlgae()).withWidget("Boolean Box").getEntry();
  }
  
  @Override
  public void periodic() {
    /** DashBoard Updates */
    hasAlgaeSender.setBoolean(hasAlgae());
  }

  /** Collect algae */
  public void collect() {
    algaeCollectorOne.set(-AffectorConstants.algaeCollectorSpeed);
    algaeCollectorTwo.set(AffectorConstants.algaeCollectorSpeed);
  }

  /** Eject algae */
  public void eject() {
    algaeCollectorOne.set(AffectorConstants.algaeCollectorSpeed);
    algaeCollectorTwo.set(-AffectorConstants.algaeCollectorSpeed);
  }

  /** Stop the affector */
  public void stopAffector() {
    algaeCollectorOne.set(0);
    algaeCollectorTwo.set(0);
  }
  
  /** Set the default configuration of the motor controllers */
  private void configMotorControllerDefaults() {
    algaeCollectorOneConfig.inverted(true);
    algaeCollectorOneConfig.idleMode(IdleMode.kBrake);

    algaeCollectorTwoConfig.inverted(true);
    algaeCollectorTwoConfig.idleMode(IdleMode.kBrake);

    configMotorControllers();
  }

  /** Update the motors controllers to their respective configurations */
  private void configMotorControllers() {
    algaeCollectorOne.configure(algaeCollectorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeCollectorTwo.configure(algaeCollectorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** @return Whether or not the AlgaeAffector has an algae */
  public boolean hasAlgae() {
    return !algaeDetector.get();
  }
}
