package frc.robot;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SnapAnglesHelper {

     public enum FieldSnapAngles
  {
    k2024CrescendoAngles(0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0),
    k2025ReefscapeAngles(0.0, 60.0, 90.0, 120.0, 180.0, 240.0, 270.0, 300.0);


    private ArrayList<Rotation2d> mAngles;
    private FieldSnapAngles(double... snapAngles)
    {
      mAngles = new ArrayList<>(snapAngles.length);
      for(double angle:snapAngles)
      {
        mAngles.add(Rotation2d.fromDegrees(angle + 90));
      }
    }

    public Rotation2d getNearestSnapAngle(double inputAngleRadians)
    {
      if(inputAngleRadians < 0)
      {
        inputAngleRadians += 2 * Math.PI;
      }
      if(inputAngleRadians > 2 * Math.PI)
      {
        inputAngleRadians -= 2* Math.PI;
      }
      Rotation2d inputRotation2d = Rotation2d.fromRadians(inputAngleRadians);

      Rotation2d currentBestAngle = mAngles.get(0);
      Rotation2d minDifference = getAngularDistance(inputRotation2d,mAngles.get(0));
      for(Rotation2d angle : mAngles)
      {
        Rotation2d diff = getAngularDistance(inputRotation2d, angle);
        if(Math.abs(diff.getDegrees()) < Math.abs(minDifference.getDegrees()))
        {
          currentBestAngle = angle;
          minDifference = diff;
        }
      }
      return currentBestAngle;
    }

    private Rotation2d getAngularDistance(Rotation2d a, Rotation2d b){
      boolean isABigger = a.getDegrees() - b.getDegrees() > 0;
      Rotation2d biggerAngle = isABigger? a:b;
      Rotation2d wrapAngle = Rotation2d.fromDegrees(biggerAngle.getDegrees() - 360.0);
      Rotation2d posibility1;
      Rotation2d posibility2;
      if(isABigger){
        posibility1 = Rotation2d.fromDegrees(a.getDegrees() - b.getDegrees());
        posibility2 = Rotation2d.fromDegrees(wrapAngle.getDegrees() - b.getDegrees());

      }
      else{
        posibility1 = Rotation2d.fromDegrees(b.getDegrees() - a.getDegrees());
        posibility2 = Rotation2d.fromDegrees(wrapAngle.getDegrees() - a.getDegrees());
      }
      if(Math.abs(posibility1.getDegrees()) < Math.abs(posibility2.getDegrees()))
      {
        return posibility1;
      }
      else
      {
        return posibility2;
      }
    }
  }

  private FieldSnapAngles mSnapAngles;

  public SnapAnglesHelper(FieldSnapAngles snapAngles)
  {
    mSnapAngles = snapAngles;
  }

  public DoubleSupplier getXDoubleSupplier(DoubleSupplier xInput, DoubleSupplier yInput){
    return new DoubleSupplier(){

      @Override
      public double getAsDouble() {
        Translation2d inputTranslation2d = new Translation2d(xInput.getAsDouble(), yInput.getAsDouble());
        double dist = Math.abs(inputTranslation2d.getDistance(Translation2d.kZero));
        if(dist == 0) return 0.0;
        return new Translation2d(dist, mSnapAngles.getNearestSnapAngle(inputTranslation2d.getAngle().getRadians())).getX();
      }
    };
  }

  public DoubleSupplier getYDoubleSupplier(DoubleSupplier xInput, DoubleSupplier yInput){
    return new DoubleSupplier(){

        @Override
        public double getAsDouble() {
          Translation2d inputTranslation2d = new Translation2d(xInput.getAsDouble(), yInput.getAsDouble());
          double dist = Math.abs(inputTranslation2d.getDistance(Translation2d.kZero));
          if(dist == 0) return 0.0;
          return new Translation2d(dist, mSnapAngles.getNearestSnapAngle(inputTranslation2d.getAngle().getRadians())).getY();
  
        }
      };
  }

}
