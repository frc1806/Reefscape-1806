// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EngageBrakeAtDesiredPosition extends Command {
  boolean hasEngaged;
  Timer mTimer;
  /** Creates a new EngageBrake. */
  public EngageBrakeAtDesiredPosition() {
    addRequirements(Elevator.GetInstance());
    hasEngaged = false;
    mTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Elevator.GetInstance().isAtPosition())
    {
      Elevator.GetInstance().engageParkingBrake();
      hasEngaged = true;
      mTimer.start();
    }
    else{
      Elevator.GetInstance().disengageParkingBrake();
      mTimer.stop();
      mTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Brake engaged finished, interrupted?" + interrupted);
    if(!interrupted) Elevator.GetInstance().stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Elevator.GetInstance().isBrakeEngaged() && hasEngaged) && mTimer.hasElapsed(ElevatorConstants.ELEVATOR_PARK_SERVO_LOCK_UNLOCK_TIME);
  }
}
