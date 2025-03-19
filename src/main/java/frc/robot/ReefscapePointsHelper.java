package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ReefscapePointsHelper {

  private SwerveSubsystem mSwerve;
  private Map<Pose2d, Command> redAllianceReefPoses;
  private Map<Pose2d, Command> blueAllianceReefPoses;


  public ReefscapePointsHelper(SwerveSubsystem swerve)
  {
    mSwerve = swerve;
  }
    public Translation2d getCenterOfReef()
    {
        if(!isRedAlliance())
        {
            return new Translation2d(4.5, 4.0); //TODO: Take real measurements
        }
        else
        {
            return new Translation2d(13.0, 4.0); //TODO: Take real measurements
        }

    }

      private static boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public Command getProcessorPathCommand()
  {
     if(!isRedAlliance())
     {
        return mSwerve.driveToPose(new Pose2d(new Translation2d(6.07, 0.56), Rotation2d.fromDegrees(-90)));
     }
     else
     {
        return mSwerve.driveToPose(new Pose2d(new Translation2d(11.566, 7.483), Rotation2d.fromDegrees(90)));
     }
  }

  public Command getNearestReefPathCommand() throws Exception
  {
    throw new Exception("Function getNearestReefPathCommand() NOT IMPLEMENTED YET"); //TODO: Implement
  }
}
