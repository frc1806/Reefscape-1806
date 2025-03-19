// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swat.lib;

/** Add your docs here. */
public class Range{
    private double mMin;
    private double mMax;

    public Range(double min, double max)
    {
      assert(min < max);
      mMax = max;
      mMin = min;
    }

    public double getMin(){
      return mMin;
    }

    public double getMax(){
      return mMax;
    }

    public double getMiddle(){
        return (mMin + mMax) / 2.0;
    }

    public double getClosestToGoal(double goal)
    {
        if(goal < mMin){
            return mMin;
        }
        if(goal > mMax)
        {
            return mMax;
        }
        return goal;
    }

    public boolean isInRage(double in){
      return in >= mMin && in <= mMax;
    }

    public int cmpRange(double in){
      if(in < mMin) return -1;
      if(in > mMax) return 1;
      return 0;
    }
  }
