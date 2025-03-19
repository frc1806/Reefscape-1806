// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PresetClawPositions;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TheClawConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TheClaw;
import swat.lib.Range;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawToPosition extends Command {
  private PresetClawPositions mGoalPosition;
  private static final Range homeSafeAngleRange = new Range(-2.0, 10.0);
  private static final Range homeUnsafeAngleRange = new Range(10.0000000001,100.0);
  private static final Range homeLimitClawHeightRange = new Range(ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_HEIGHT_FOR_SAFE_CLAW_MOVE);
  private boolean mNeedsFlip = false;

  /** Creates a new ClawToPosition. */
  public ClawToPosition(PresetClawPositions clawPosition) {
    mGoalPosition = clawPosition;
    addRequirements(Elevator.GetInstance(), TheClaw.GetInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(homeLimitClawHeightRange.isInRage(mGoalPosition.getElevatorHeight()))
    {
      if(homeSafeAngleRange.isInRage(TheClaw.GetInstance().getAngle()) ^ homeSafeAngleRange.isInRage(mGoalPosition.getTheClawAngle()))
      {
        mNeedsFlip = true;
      }
      else
      {
        mNeedsFlip = false;
      }
    }
    else
    {
      mNeedsFlip = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mNeedsFlip){
      //DO A FLIP
      Elevator.GetInstance().GoToPosition(ElevatorConstants.ELEVATOR_HEIGHT_FOR_SAFE_CLAW_MOVE + 2.0);
      TheClaw.GetInstance().goToPosition(getSafeClawAngleRangeForElevatorHeight(Elevator.GetInstance().GetPosition()).getClosestToGoal(mGoalPosition.getTheClawAngle()));
      if(Elevator.GetInstance().isAboveHeight(ElevatorConstants.CLAW_ANGLE_FOR_SAFE_ELEVATOR_MOVE) && homeUnsafeAngleRange.cmpRange(TheClaw.GetInstance().getAngle()) == homeUnsafeAngleRange.cmpRange(mGoalPosition.getTheClawAngle()))
      {
        mNeedsFlip = false;
      }
    }
    else{
      Elevator.GetInstance().GoToPosition(getSafeElevatorRangeForClawAngle(TheClaw.GetInstance().getAngle()).getClosestToGoal(mGoalPosition.getElevatorHeight()));
      TheClaw.GetInstance().goToPosition(getSafeClawAngleRangeForElevatorHeight(Elevator.GetInstance().GetPosition()).getClosestToGoal(mGoalPosition.getTheClawAngle()));
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted)
    {
      RobotContainer.S_MOST_RECENT_ACHIEVED_CLAW_POSITION = mGoalPosition; //store this so it can be used for the scoring sequence if applicable
    }
  }

  /**
   * 
   * @param angle
   * @return
   */
  private Range getSafeElevatorRangeForClawAngle(double angle)
  {
    if(angle > TheClawConstants.OVER_THE_TOP_ANGLE)
    {
      return new Range(ElevatorConstants.ELEVATOR_HEIGHT_FOR_SAFE_OVER_THE_TOP , ElevatorConstants.ELEVATOR_MAX_HEIGHT);
    }
    switch(homeSafeAngleRange.cmpRange(angle)){
      default:
      case -1:
        return new Range(Elevator.GetInstance().GetPosition(), Elevator.GetInstance().GetPosition()); //We are scraping the claw into the elevator, don't allow it to move
      case 0:
        return new Range(ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT); //Safe for home, can be any height
      case 1:
        if(homeUnsafeAngleRange.isInRage(angle))
        {
          return new Range(ElevatorConstants.ELEVATOR_HEIGHT_FOR_SAFE_CLAW_MOVE, ElevatorConstants.ELEVATOR_MAX_HEIGHT); // Don't put the claw through the battery
        }
        else
        {
          return new Range(ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT);// flipped
        }
        
    }
  }

  /**
   * 
   * @param height
   * @return
   */
  private Range getSafeClawAngleRangeForElevatorHeight(double height)
  {
    if(height > ElevatorConstants.ELEVATOR_HEIGHT_FOR_SAFE_OVER_THE_TOP)
    {
      return new Range(-1.0, 270.0);
    }
    else
    {
      switch(homeLimitClawHeightRange.cmpRange(height))
      {
        default:
        case -1:
        return new Range(TheClaw.GetInstance().getAngle(), TheClaw.GetInstance().getAngle());
        case 0:
        return homeSafeAngleRange;
        case 1:
          return new Range( -1.0, TheClawConstants.OVER_THE_TOP_ANGLE);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Elevator.GetInstance().isAtArbitraryPosition(mGoalPosition.getElevatorHeight()) && TheClaw.GetInstance().isAtArbitraryPosition(mGoalPosition.getTheClawAngle());
  }

  

}
