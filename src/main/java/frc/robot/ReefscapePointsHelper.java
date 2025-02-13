package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefscapePointsHelper {
    public static Translation2d getCenterOfReef()
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

  public static Pose2d getProcessorPose()
  {
     if(!isRedAlliance())
     {
        return new Pose2d(new Translation2d(6.07, 0.56), Rotation2d.fromDegrees(-90));
     }
     else
     {
        return new Pose2d(new Translation2d(11.566, 7.483), Rotation2d.fromDegrees(90));
     }
  }
}
