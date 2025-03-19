// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeSpitOut extends Command {
  /** Creates a new CoralIntakeSpitOut. */
  public CoralIntakeSpitOut() {
    addRequirements(CoralIntake.GetInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CoralIntake.GetInstance().reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CoralIntake.GetInstance().reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralIntake.GetInstance().retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
