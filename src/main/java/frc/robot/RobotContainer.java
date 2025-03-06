// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANdi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.SnapAnglesHelper.FieldSnapAngles;
import frc.robot.commands.ClawsToPresetPosition;
import frc.robot.commands.EverythingToHome;
import frc.robot.commands.algaeclaw.AlgaeClawRunRollersIn;
import frc.robot.commands.algaeclaw.AlgaeClawRunRollersOut;
import frc.robot.commands.algaeclaw.AlgaeClawToAngle;
import frc.robot.commands.coralclaw.CoralClawCloseClaw;
import frc.robot.commands.coralclaw.CoralClawOpenClaw;
import frc.robot.commands.coralclaw.CoralClawRunRollersOut;
import frc.robot.commands.coralclaw.CoralClawToAngle;
import frc.robot.commands.coralintake.CoralIntakeIntake;
import frc.robot.commands.coralintake.CoralIntakeRetract;
import frc.robot.commands.coralintake.CoralIntakeSpitOut;
import frc.robot.commands.elevator.DisengageBrake;
import frc.robot.commands.elevator.ElevatorMoveSequence;
import frc.robot.commands.elevator.ElevatorToHeight;
import frc.robot.commands.elevator.EngageBrakeAtDesiredPosition;
import frc.robot.commands.swervedrive.WaitForBackAway;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.StopDrive;
import frc.robot.commands.utility.GetAndRunCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public enum TEST_MODES{
    SwerveTest,
    ElevatorBrakeTest,
    ElevatorMotionTest,
    ElevatorFullTest,
    CoralIntakeArmTest,
    CoralClawAngleTest,
    CoralClawOpenCloseTest,
    AlgaeClawAngleTest,
    PlayMusic
  }

  public enum Songs{
    TheThingThatShouldNotBe,
    StarSpangledBanner,
    GoRollingAlong
  }

  public static final SendableChooser<TEST_MODES> TEST_MODE_CHOOSER = new SendableChooser<>();
  public static final SendableChooser<Songs> SONG_CHOOSER = new SendableChooser<>();

  public static final CANdi S_CARRIAGE_CANDI = new CANdi(RobotMap.ELEVATOR_CANDI_ID);

  public static final Command CORAL_INTAKE_SEQUENCE = new SequentialCommandGroup(new ClawsToPresetPosition(PresetClawPositions.kHome), new CoralClawOpenClaw(), new CoralIntakeIntake(), new CoralClawCloseClaw());
  public static final Command ALGAE_INTAKE_SEQUENCE = new SequentialCommandGroup( new AlgaeClawRunRollersIn(), new WaitForBackAway(), new ClawsToPresetPosition(PresetClawPositions.kHome));
  public static final Command CORRAL_SCORE_SEQUENCE = new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(.2), new CoralClawRunRollersOut()), new WaitForBackAway(), new ClawsToPresetPosition(PresetClawPositions.kHome)); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private final Orchestra orchestra = new Orchestra();

  private final ReefscapePointsHelper pointsHelper = new ReefscapePointsHelper(drivebase);
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  AutoBuilder mAutoBuilder = new AutoBuilder();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SnapAnglesHelper snapAnglesHelper = new SnapAnglesHelper(FieldSnapAngles.k2025ReefscapeAngles);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(snapAnglesHelper.getXDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1), snapAnglesHelper.getYDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1)).headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(1.0)
                                                               .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy().withControllerHeadingAxis(snapAnglesHelper.getXDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1), snapAnglesHelper.getYDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1)).headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    setupNamedCommands();
    
    for(TEST_MODES testMode: TEST_MODES.values())
    {
      TEST_MODE_CHOOSER.addOption(testMode.name(), testMode);
    }
    SmartDashboard.putData("Test Mode", TEST_MODE_CHOOSER);
    SmartDashboard.putData("Song", SONG_CHOOSER);

    Elevator.GetInstance().addToOrchestra(orchestra);
    drivebase.addToOrchestra(orchestra);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedDirectAngleSim);
    //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      if(TEST_MODE_CHOOSER.getSelected() != null)
      {
        if(TEST_MODE_CHOOSER.getSelected() != TEST_MODES.SwerveTest)
        {
          drivebase.setDefaultCommand(new StopDrive());
        }
        switch(TEST_MODE_CHOOSER.getSelected())
        {
          case AlgaeClawAngleTest:
            Command algaeClawSetupBase = new ClawsToPresetPosition(PresetClawPositions.kClawMotionTest);
            driverXbox.a().onTrue(algaeClawSetupBase.andThen(new AlgaeClawToAngle(0.0)));
            driverXbox.b().onTrue(algaeClawSetupBase.andThen(new AlgaeClawToAngle(45.0)));
            driverXbox.y().onTrue(algaeClawSetupBase.andThen(new AlgaeClawToAngle(120.0)));
            driverXbox.x().onTrue(algaeClawSetupBase.andThen(new AlgaeClawToAngle(200.0)));
            break;
          case CoralClawAngleTest:
            Command coralClawSetupBase = new ClawsToPresetPosition(PresetClawPositions.kClawMotionTest);
            driverXbox.a().onTrue(coralClawSetupBase.andThen(new CoralClawToAngle(0.0)));
            driverXbox.b().onTrue(coralClawSetupBase.andThen(new CoralClawToAngle(120.0)));
            driverXbox.y().onTrue(coralClawSetupBase.andThen(new CoralClawToAngle(180.0)));
            break;
          case CoralClawOpenCloseTest:
            Command coralClawOpenSetupBase = new ClawsToPresetPosition(PresetClawPositions.kClawMotionTest);
            driverXbox.a().onTrue(coralClawOpenSetupBase.andThen(new CoralClawOpenClaw()));
            driverXbox.b().onTrue(coralClawOpenSetupBase.andThen(new CoralClawCloseClaw()));
            break;
          case CoralIntakeArmTest:
            driverXbox.a().onTrue(new CoralIntakeIntake());
            driverXbox.b().onTrue(new CoralIntakeRetract());
            break;
          case ElevatorBrakeTest:
            driverXbox.a().onTrue(new EngageBrakeAtDesiredPosition());
            driverXbox.b().onTrue(new DisengageBrake());
            break;
          case ElevatorFullTest:
            driverXbox.a().onTrue(new ElevatorMoveSequence(PresetClawPositions.kHome.getElevatorHeight()));
            driverXbox.b().onTrue(new ElevatorMoveSequence(PresetClawPositions.kCoralL3.getElevatorHeight()));
            driverXbox.y().onTrue(new ElevatorMoveSequence(PresetClawPositions.kCoralL4.getElevatorHeight()));
            break;
          case ElevatorMotionTest:
            driverXbox.a().onTrue(new ElevatorToHeight(PresetClawPositions.kHome.getElevatorHeight()));
            driverXbox.b().onTrue(new ElevatorToHeight(PresetClawPositions.kCoralL3.getElevatorHeight()));
            driverXbox.y().onTrue(new ElevatorToHeight(PresetClawPositions.kCoralL4.getElevatorHeight()));
            break;
          case SwerveTest:
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
  
            driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            driverXbox.leftBumper().onTrue(Commands.none());
            driverXbox.rightBumper().onTrue(Commands.none());
            break;
          case PlayMusic:
            if(SONG_CHOOSER.getSelected() != null)
            {
              switch(SONG_CHOOSER.getSelected())
              {
                case GoRollingAlong:
                  orchestra.loadMusic("gorollingalong.chrp");
                  break;
                case StarSpangledBanner:
                  orchestra.loadMusic("starspangledbanner.chrp");
                  break;
                default: //intentional no break to default to thing that should not be
                case TheThingThatShouldNotBe:
                  orchestra.loadMusic("thethingthatshouldnotbe.chrp");
                  break;
              }
              orchestra.play();
            }

          default:
            break;
          
        }
      }
      

    } else
    {
      driverXbox.povDown().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      //driverXbox.b().whileTrue(
          //drivebase.driveToPose(()->ReefscapePointsHelper.getProcessorPose()));
      driverXbox.start().onTrue(new ClawsToPresetPosition(PresetClawPositions.kHome));
      driverXbox.rightBumper().and(driverXbox.a()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kAlgaeL2));
      driverXbox.rightBumper().negate().and(driverXbox.a()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kCoralL2));
      driverXbox.rightBumper().and(driverXbox.b()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kAlgaeL3));
      driverXbox.rightBumper().negate().and(driverXbox.b()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kCoralL3));
      driverXbox.rightBumper().and(driverXbox.y()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kAlgaeNetForward));
      driverXbox.rightBumper().negate().and(driverXbox.y()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kCoralL4));
      driverXbox.rightBumper().and(driverXbox.x()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kAlgaeProcessBackward));
      driverXbox.rightBumper().negate().and(driverXbox.x()).onTrue(new ClawsToPresetPosition(PresetClawPositions.kCoralL1));

      driverXbox.rightTrigger().whileTrue(new CoralIntakeIntake());

      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      new Trigger(drivebase::isPanicSituation).onTrue(new EverythingToHome());
      driverXbox.povRight().whileTrue(new GetAndRunCommand(pointsHelper::getProcessorPathCommand));
      operatorXbox.a().whileTrue(new CoralIntakeSpitOut());
      operatorXbox.start().onTrue(new EverythingToHome());
      

    }

  }

  public void setupNamedCommands(){
    NamedCommands.registerCommand("HomeClaws", new ClawsToPresetPosition(PresetClawPositions.kHome));
    NamedCommands.registerCommand("L4Coral", new ClawsToPresetPosition(PresetClawPositions.kCoralL4));
    NamedCommands.registerCommand("L3Coral", new ClawsToPresetPosition(PresetClawPositions.kCoralL3));
    NamedCommands.registerCommand("L2Coral", new ClawsToPresetPosition(PresetClawPositions.kCoralL2));
    NamedCommands.registerCommand("L3Algae", new ClawsToPresetPosition(PresetClawPositions.kAlgaeL3));
    NamedCommands.registerCommand("L2Algae", new ClawsToPresetPosition(PresetClawPositions.kAlgaeL2));
    NamedCommands.registerCommand("CoralIntake", CORAL_INTAKE_SEQUENCE);
    NamedCommands.registerCommand("AlgaeClawIn", new AlgaeClawRunRollersIn());
    NamedCommands.registerCommand("AlgaeOut", new ParallelDeadlineGroup(new WaitCommand(200),new AlgaeClawRunRollersOut()));
    NamedCommands.registerCommand("CoralOut", new ParallelDeadlineGroup(new WaitCommand(200),new CoralClawRunRollersOut()));
    NamedCommands.registerCommand(null, closedAbsoluteDriveAdv);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    
    
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("GroundPickup");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
