package frc.robot;

import java.util.HashMap;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.Lib.AdvancedPose2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class IOConstants {
    /* DASHBOARD TABS */
    public static final ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");
    public static final ShuffleboardTab AutonTab = Shuffleboard.getTab("Auton");
    public static final ShuffleboardTab DiagnosticTab = Shuffleboard.getTab("Diagnostic");
    public static final ShuffleboardTab ConfigTab = Shuffleboard.getTab("Configuration");

    /* CONTROLLER IDS */
    public static final int driverControllerID = 0;
    public static final int operatorController1ID = 1;
    public static final int operatorController2ID = 2;

    /* BUTTON IDS */
    /* Driver */
    //Main Functions
    public static final int quickBrakeButtonID = 1; //6
    public static final int slowModeButtonID = 2; //3
    public static final int switchOrientationButtonID = 4; //4
    public static final int lockPoseButtonID = 5; //5
    public static final int straightDriveButtonID = 6; //2
    public static final int autoDriveButtonID = 0; //1
    public static final int robotOrientButtonID = 9; //9

    //Alignments
    public static final int leftAlignButtonID = 11;
    public static final int centerAlignButtonID = 10;
    public static final int rightAlignButtonID = 12;

    /* Operator */
    //Positions
    public static final int floorPosButtonID = 1;
    public static final int processorPosButtonID = 2;
    public static final int L2AlgaePosButtonID = 5;
    public static final int L2PosButtonID = 4;
    public static final int L3AlgaePosButtonID = 7;
    public static final int L3PosButtonID = 6;
    public static final int L4PosButtonID = 8;
    public static final int HPPosButtonID = 3;

    //Affectors
    public static final int coralCollectButtonID = 1;
    public static final int coralScoreButtonID = 2;
    public static final int algaeCollectButtonID = 3;
    public static final int algaeScoreButtonID = 4;

    //Overrides
    public static final int wristOverrideButtonID = 12;
    public static final int elevatorOverrideButtonID = 12;
  }

  public class ElevatorConstants {
    public static final int elevatorMotorID = 9;
    public static final int elevatorTwoID = 10;

    public static final double startingHeight = 0;

    public static final double conversionFactor = 1;
    public static final double voltageComp = 12;

    public static final double lowerLimit = 5;
    public static final double upperLimit = 215;

    public static final double kP = .1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTolerance = 1;

    public static final double upSpeed = .4;
    public static final double downSpeed = -.4;
    public static final double overrideSpeed = .5;
    public static final double lockSpeed = .025;

    public static final double maxUpSpeed = .99;
    public static final double maxDownSpeed = -.93;
    public static final double correctionSpeed = .2;

    public static final double defaultPos = lowerLimit + 1;
    public static final double floorPos = lowerLimit + 1;
    public static final double processorPos = 32;
    public static final double L2AlgaePos = 115;
    public static final double L2Pos = 87;
    public static final double L3AlgaePos = 183;
    public static final double L3Pos = 214;
    public static final double L4Pos = 214;
    public static final double HPPos = 88;
  }

  public class AffectorConstants {
    /** CORAL */
    public static final int coralAffectorID = 11;
    public static final int coralWristID = 12;

    public static final double startingAngle = 1;
    public static final double voltageComp = 12;

    public static final double coralWristConversionFactor = (360 / 64);
    public static final double wristScoringThreshold = 60;

    public static final double coralWristDefaultPos = startingAngle;
    public static final double coralWristL23 = 19;
    public static final double coralWristL4 = 100;
    public static final double coralWristHP = 36;

    public static final double wristOverrideSpeed = .2;

    public static final double coralWristKP = .016; //.016
    public static final double coralWristKI = .000; //.000
    public static final double coralWristKD = .000; //.000
    public static final double coralWristKTolerance = .5;

    public static final double coralAffectorInSpeed = .25;
    public static final double coralAffectorOutSpeed = .75;

    public static final double maxCoralWristUpSpeed = .4;
    public static final double maxCoralWristDownSpeed = -.175;

    /** ALGAE */
    public static final int algaeCollectorOneID = 13;
    public static final int algaeCollectorTwoID = 14;

    public static final double algaeCollectorSpeed = .3;
  }

  public static final class SwerveConstants {
    // Distance between centers of right and left wheels on robot in meters
    public static final double trackWidth = Units.inchesToMeters(36); //.9144
    
    // Distance between front and back wheels on robot in meters
    public static final double wheelBase = Units.inchesToMeters(36); //.9144
    
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(new Translation2d(wheelBase / 2, trackWidth / 2),
                                  new Translation2d(wheelBase / 2, -trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, -trackWidth / 2));
  }

  public static final class DriveConstants {
    public static final double speedScaler = .25;

    public static final double maxDriveSpeed = 1;
    public static final double minDriveSpeed = .35;
    public static final double maxRotSpeed = 1;
    public static final double minRotSpeed = .3;

    public static final double maxSpeedMetersPerSecond = 4.25; //4.5 true max
    public static final double maxAccelerationMetersPerSecondSquared = 1;
    public static final double maxRotationSpeedRadiansPerSecond = Math.PI * .8;

    public static final double xyDeadband = .1;
    public static final double zDeadband = .4;

    public static final double ksVolts = 5;
    public static final double kvVoltSecondsPerMeter = 4;
    public static final double kaVoltSecondsSquaredPerMeter = 1;

    public static final boolean gyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final double moduleTurningController = .5;
    public static final double moduleDriveController = .75;
    public static final double moduleDriveSlewRate = 2;

    public static final int frontLeftDriveMotorPort = 1;
    public static final int frontRightDriveMotorPort = 3;
    public static final int backLeftDriveMotorPort = 7;
    public static final int backRightDriveMotorPort = 5;

    public static final int frontLeftTurningMotorPort = 2;
    public static final int frontRightTurningMotorPort = 4;
    public static final int backLeftTurningMotorPort = 8;
    public static final int backRightTurningMotorPort = 6;
    
    public static final int frontLeftTurningEncoderPort = 0;
    public static final int frontRightTurningEncoderPort = 1;
    public static final int backLeftTurningEncoderPort = 3;
    public static final int backRightTurningEncoderPort = 2;

    public static final boolean frontLeftTurningMotorReversed = false;    
    public static final boolean frontRightTurningMotorReversed = false;
    public static final boolean backLeftTurningMotorReversed = false;
    public static final boolean backRightTurningMotorReversed = false;

    public static final boolean frontLeftDriveMotorReversed = false;    
    public static final boolean frontRightDriveMotorReversed = false;
    public static final boolean backLeftDriveMotorReversed = false;
    public static final boolean backRightDriveMotorReversed = false;
    
    public static final boolean frontLeftAbsReversed = false;    
    public static final boolean frontRightAbsReversed = false;
    public static final boolean backLeftAbsReversed = false;
    public static final boolean backRightAbsReversed = false;

    public static final double frontLeftAnalogEncoderOffset = 4.69;  
    public static final double frontRightAnalogEncoderOffset = 75.88;
    public static final double backLeftAnalogEncoderOffset = 162.39;
    public static final double backRightAnalogEncoderOffset = 63.79;

    public static final double maxModuleAngularSpeedDegreesPerSecond = 360;
    public static final double maxModuleAngularAccelerationDegreesPerSecondSquared = 360;

    public static final double encoderCPR = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(3.8125);
    public static final double driveGearRatio = 1 / 6.75;
    public static final double driveMotorConversionFactor = 
      // Assumes the encoders are directly mounted on the wheel shafts
      (wheelDiameterMeters * Math.PI) / (double) encoderCPR * driveGearRatio;

    public static final double angleGearRatio = (150 / 7);
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    public static final double voltageComp = 12.0;
    public static final int angleContinuousCurrentLimit = 20;
  
    public static final double angleKP = 0.01; //.01
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;
    public static final double kMaxOutput = 0.95;
    public static final double kTolerance = .5;
  
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  }

  public class FieldConstants {
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.052;

    public static final double autonLineDistance = Units.inchesToMeters(297.5);

    public static final AdvancedPose2D blueReefCenterPos = new AdvancedPose2D(4.48945, fieldWidth / 2, Rotation2d.fromDegrees(0));

    public static final AdvancedPose2D blueLeftStartAlgae = new AdvancedPose2D(Units.inchesToMeters(48), 
                                                                               blueReefCenterPos.getY() + Units.inchesToMeters(72), 
                                                                               0);
    public static final AdvancedPose2D blueCenterStartAlgae = new AdvancedPose2D(Units.inchesToMeters(48), 
                                                                                 blueReefCenterPos.getY(), 0);
    public static final AdvancedPose2D blueRightStartAlgae = new AdvancedPose2D(Units.inchesToMeters(48), 
                                                                                blueReefCenterPos.getY() - Units.inchesToMeters(72), 
                                                                                0);

    public static final AdvancedPose2D blueProcessor = new AdvancedPose2D(autonLineDistance - Units.inchesToMeters(61.76), 
                                                                          SwerveConstants.trackWidth / 2, -90);
    public static final AdvancedPose2D redProcessor = blueProcessor.flipBoth();

    public static final double coralStationWidth = Units.inchesToMeters(79.76);
    public static final double coralStationTotalX = autonLineDistance - Units.inchesToMeters(231.66); //65.84
    public static final double coralStationTotalY = Math.sqrt(Math.pow(coralStationWidth, 2) - Math.pow(coralStationTotalX, 2));
    public static final double coralStationXMid = coralStationTotalX / 2;
    public static final double coralStationYMid = coralStationTotalY / 2;
    public static final double coralStationAngle = Units.radiansToDegrees(Math.atan(coralStationTotalX / coralStationTotalY));
    public static final double coralStationDesiredHeading = 180 - coralStationAngle;
  }

  public class AutoAimConstants{
    public static enum Position {floor, processor, L2algae, L2, L3algae, L3, L4, HP};
    public static enum ReefStation {front, frontRight, backRight, back, backLeft, frontLeft};
    public static enum Alignment {left, center, right, blank};

    public static final HashMap<Position, double[]> positionValues = new HashMap<Position, double[]> () {{
      put(Position.floor, new double[] {ElevatorConstants.floorPos, AffectorConstants.coralWristDefaultPos});
      put(Position.processor, new double[] {ElevatorConstants.processorPos, AffectorConstants.coralWristDefaultPos});
      put(Position.L2algae, new double[] {ElevatorConstants.L2AlgaePos, AffectorConstants.coralWristDefaultPos});
      put(Position.L2, new double[] {ElevatorConstants.L2Pos, AffectorConstants.coralWristL23});
      put(Position.L3algae, new double[] {ElevatorConstants.L3AlgaePos, AffectorConstants.coralWristDefaultPos});
      put(Position.L3, new double[] {ElevatorConstants.L3Pos, AffectorConstants.coralWristL23});
      put(Position.L4, new double[] {ElevatorConstants.L4Pos, AffectorConstants.coralWristL4});
      put(Position.HP, new double[] {ElevatorConstants.HPPos, AffectorConstants.coralWristHP});
    }};
            
    public static final double centerOfReefToRobotDistance = Units.inchesToMeters(32.75) + SwerveConstants.trackWidth / 2;

    public static final double coralAffectorOffsetFromRobotCenter = Units.inchesToMeters(1.5);
    public static final double leftCoralReefOffset = -(Units.inchesToMeters(2) + coralAffectorOffsetFromRobotCenter);
    public static final double rightCoralReefOffset = Units.inchesToMeters(2) - coralAffectorOffsetFromRobotCenter;
    public static final double algaePosBackset = Units.inchesToMeters(9); 
    public static final double coralPosBackset = Units.inchesToMeters(3);

    public static final double LLDefaultOffsetDegrees = 2.3;
    public static final double LCToBumperEdgeOffsetMeters = Units.inchesToMeters(4.85);

    public static final double coralStationToRobotDistance = SwerveConstants.trackWidth / 2 + coralPosBackset;
    public static final double coralStationSideOffsetDistance = Units.inchesToMeters(76) / 4 - coralAffectorOffsetFromRobotCenter;

    public static final HashMap<Alignment, Double> offsetFromAlignment = new HashMap<Alignment, Double> () {{
      put(Alignment.left, leftCoralReefOffset);
      put(Alignment.center, 0.0);
      put(Alignment.right, rightCoralReefOffset);
      put(Alignment.blank, 0.0);
    }};

    public static final double[] reefStationAngles = {0, 60, 120, 180, -180, -120, -60};

    public static final HashMap<ReefStation, AdvancedPose2D> blueReef = new HashMap<ReefStation, AdvancedPose2D> () {{
      put(ReefStation.front, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(0), new Translation2d(0, -centerOfReefToRobotDistance), Rotation2d.fromDegrees(0)));
      put(ReefStation.frontRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-120), new Translation2d(0, centerOfReefToRobotDistance), Rotation2d.fromDegrees(60)));
      put(ReefStation.backRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-60), new Translation2d(0, centerOfReefToRobotDistance), Rotation2d.fromDegrees(120)));
      put(ReefStation.back, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(0), new Translation2d(0, centerOfReefToRobotDistance), Rotation2d.fromDegrees(180)));
      put(ReefStation.backLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(60), new Translation2d(0, centerOfReefToRobotDistance), Rotation2d.fromDegrees(-120)));
      put(ReefStation.frontLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(120), new Translation2d(0, centerOfReefToRobotDistance), Rotation2d.fromDegrees(-60)));
    }};

    public static final HashMap<ReefStation, AdvancedPose2D> redReef = new HashMap<ReefStation, AdvancedPose2D> () {{
      put(ReefStation.front, blueReef.get(ReefStation.front).flipBoth());
      put(ReefStation.frontRight, blueReef.get(ReefStation.frontRight).flipBoth());
      put(ReefStation.backRight, blueReef.get(ReefStation.backRight).flipBoth());
      put(ReefStation.back, blueReef.get(ReefStation.back).flipBoth());
      put(ReefStation.backLeft, blueReef.get(ReefStation.backLeft).flipBoth());
      put(ReefStation.frontLeft, blueReef.get(ReefStation.frontLeft).flipBoth());
    }};

    public static final HashMap<Double, ReefStation> blueReefStationFromAngle = new HashMap<Double, ReefStation> () {{
      put(0.0, ReefStation.front);
      put(60.0, ReefStation.frontRight);
      put(120.0, ReefStation.backRight);
      put(180.0, ReefStation.back);
      put(-180.0, ReefStation.back);
      put(-120.0, ReefStation.backLeft);
      put(-60.0, ReefStation.frontLeft);
    }};

    public static final HashMap<Double, ReefStation> redReefStationFromAngle = new HashMap<Double, ReefStation>() {{
      put(0.0, ReefStation.back);
      put(60.0, ReefStation.backLeft);
      put(120.0, ReefStation.frontLeft);
      put(180.0, ReefStation.front);
      put(-180.0, ReefStation.front);
      put(-120.0, ReefStation.frontRight);
      put(-60.0, ReefStation.backRight);
    }};

    public static final HashMap<ReefStation, Double> angleFromReefStation = new HashMap<ReefStation, Double> () {{
      put(ReefStation.front, 0.0);
      put(ReefStation.frontRight, 60.0);
      put(ReefStation.backRight, 120.0);
      put(ReefStation.back, 180.0);
      put(ReefStation.backLeft, -120.0);
      put(ReefStation.frontLeft, -60.0);
    }};

    public static final HashMap<Double, AdvancedPose2D> reefStationPoseFromAprilTagID = new HashMap<Double, AdvancedPose2D> () {{
      put(18., blueReef.get(ReefStation.front));
      put(17., blueReef.get(ReefStation.frontRight));
      put(22., blueReef.get(ReefStation.backRight));
      put(21., blueReef.get(ReefStation.back));
      put(20., blueReef.get(ReefStation.backLeft));
      put(19., blueReef.get(ReefStation.frontLeft));

      put(7., redReef.get(ReefStation.front));
      put(8., redReef.get(ReefStation.frontRight));
      put(9., redReef.get(ReefStation.backRight));
      put(10., redReef.get(ReefStation.back));
      put(11., redReef.get(ReefStation.backLeft));
      put(6., redReef.get(ReefStation.frontLeft));
    }};

    public static final HashMap<Double, ReefStation> reefStationFromAprilTagID = new HashMap<Double, ReefStation> () {{
      put(18., ReefStation.front);
      put(17., ReefStation.frontRight);
      put(22., ReefStation.backRight);
      put(21., ReefStation.back);
      put(20., ReefStation.backLeft);
      put(19., ReefStation.frontLeft);

      put(7., ReefStation.front);
      put(8., ReefStation.frontRight);
      put(9., ReefStation.backRight);
      put(10., ReefStation.back);
      put(11., ReefStation.backLeft);
      put(6., ReefStation.frontLeft);

      put(0., ReefStation.front);
    }};

    public static final HashMap<ReefStation, Double> blueReefIDsFromStation = new HashMap<ReefStation, Double>() {{
      put(ReefStation.front, 18.);
      put(ReefStation.frontRight, 17.);
      put(ReefStation.backRight, 22.);
      put(ReefStation.back, 21.);
      put(ReefStation.backLeft, 20.);
      put(ReefStation.frontLeft, 19.);
    }};

    public static final HashMap<ReefStation, Double> redReefIDsFromStation = new HashMap<ReefStation, Double>() {{
      put(ReefStation.front, 7.);
      put(ReefStation.frontRight, 8.);
      put(ReefStation.backRight, 9.);
      put(ReefStation.back, 10.);
      put(ReefStation.backLeft, 11.);
      put(ReefStation.frontLeft, 6.);
    }};

    public static final AdvancedPose2D blueLeftCoralStationPos = new AdvancedPose2D(FieldConstants.coralStationXMid, 
                                                                                    FieldConstants.fieldWidth - FieldConstants.coralStationYMid, 
                                                                                    FieldConstants.coralStationDesiredHeading)
                                                                                    .withRobotRelativeTransformation(
                                                                                      new Translation2d(0, -coralStationToRobotDistance));
    public static final AdvancedPose2D blueRightCoralStationPos = blueLeftCoralStationPos.verticallyFlip();
    public static final AdvancedPose2D redLeftCoralStationPos = blueLeftCoralStationPos.flipBoth();
    public static final AdvancedPose2D redRightCoralStationPos = blueRightCoralStationPos.flipBoth();
    
    public static final double transkP = 2; //1.5
    public static final double transkI = 0; //.013
    public static final double transkD = 0; //0
    public static final double transkTolerance = .025;

    public static final double turnkP = 4.5; //4.5
    public static final double turnkI = .355; //.355
    public static final double turnkD = 0; //0
    public static final double turnkTolerance = .03;

    public static final Vector<N3> poseEstimateOdometryStdDev = VecBuilder.fill(.1, .1, Units.degreesToRadians(.2));
    public static final Vector<N3> poseEstimateVisionStdDev = VecBuilder.fill(.1, .1, Units.degreesToRadians(3));

    public static final double maxPIDDriveSpeed = .7;
    public static final double maxPIDRot = Math.PI/2;
  }

  public class AutonConstants {
    public static final double coralScoreTime = 1;
    public static final double alageScoreTime = .3;

    public static final double awayFromReefTime = 1;

    public static final double inRangeThreshold = 2.15; //2.15

    public static final AdvancedPose2D initialPoseBlueRight = new AdvancedPose2D(FieldConstants.autonLineDistance,
                                                                                 SwerveConstants.trackWidth / 2,
                                                                                 90);
    public static final AdvancedPose2D initialPoseRedRight = initialPoseBlueRight.flipBoth();

    public static final AdvancedPose2D initialPoseBlueLeft = new AdvancedPose2D(FieldConstants.autonLineDistance, 
                                                                                FieldConstants.fieldWidth - 
                                                                                        (SwerveConstants.trackWidth / 2), 
                                                                                -90);
    public static final AdvancedPose2D initialPoseRedLeft = initialPoseBlueLeft.flipBoth();

    public static final AdvancedPose2D initialPoseBlueBack = new AdvancedPose2D(FieldConstants.autonLineDistance, 
                                                                                FieldConstants.blueReefCenterPos.getY(),
                                                                                180);
    public static final AdvancedPose2D initialPoseRedBack = initialPoseBlueBack.flipBoth();

    public static final AdvancedPose2D initialPoseDOLBlueRight = initialPoseBlueRight.rotateBy(Rotation2d.fromDegrees(90));
    public static final AdvancedPose2D initialPoseDOLBlueLeft = initialPoseBlueLeft.rotateBy(Rotation2d.fromDegrees(-90));
    public static final AdvancedPose2D initialPoseDOLRedRight = initialPoseDOLBlueRight.flipBoth();
    public static final AdvancedPose2D initialPoseDOLRedLeft = initialPoseDOLBlueLeft.flipBoth();

    public static final AdvancedPose2D customInitialPose = new AdvancedPose2D();
  }

  public class SensorConstants {
    /** LIMELIGHT */
    public static final String limeLightName = "limelight";
    //                                                     {forward, right, up, roll, pitch, yaw}
    public static final double[] limelightRobotSpacePose = {Units.inchesToMeters(1.875), 
                                                           -Units.inchesToMeters(5.875), 
                                                            Units.inchesToMeters(33.6875), 
                                                            0, 0, 0};

    /** BEAM BREAKS */
    public static final int coralBreakID = 0;
    public static final int algaeBreakID = 1;
    
    /** WRIST LIMIT SWITCH */
    public static final int wristResetterID = 2;
  }
}