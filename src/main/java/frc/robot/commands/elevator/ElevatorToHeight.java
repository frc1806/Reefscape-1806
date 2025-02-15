// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToHeight extends Command {

  private double mHeight;
  /** Creates a new ElevatorToHeight. */
  public ElevatorToHeight(double height) {
    addRequirements(Elevator.GetInstance());
    if(height < ElevatorConstants.ELEVATOR_MIN_HEIGHT) height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
    if(height > ElevatorConstants.ELEVATOR_MAX_HEIGHT) height = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    mHeight = height;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.GetInstance().GoToPosition(mHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("elevator to height finished, interrupted?" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Elevator.GetInstance().isAtPosition();
  }
}
