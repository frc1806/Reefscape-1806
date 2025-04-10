// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TheClawRunRollersIn extends Command {
  /** Creates a new AlgaeClawRunRollersIn. */
  public TheClawRunRollersIn() {
    addRequirements(ClawRoller.GetInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClawRoller.GetInstance().runRollersIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClawRoller.GetInstance().runRollersIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) ClawRoller.GetInstance().stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClawRoller.GetInstance().hasGamePiece();
  }
}
