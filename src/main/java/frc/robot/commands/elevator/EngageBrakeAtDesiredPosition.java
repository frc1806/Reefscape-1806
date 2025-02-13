// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EngageBrakeAtDesiredPosition extends Command {
  boolean hasEngaged;
  /** Creates a new EngageBrake. */
  public EngageBrakeAtDesiredPosition() {
    addRequirements(Elevator.GetInstance());
    hasEngaged = false;
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
    }
    else{
      Elevator.GetInstance().disengageParkingBrake();
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
    return Elevator.GetInstance().isBrakeEngaged() && hasEngaged;
  }
}
