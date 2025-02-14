// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetAndRunCommand extends Command {
  /** Creates a new GetAndRunCommand. This command is used to evaluate what command should be run at the time the trigger is evaluated by using a command supplier. */
  Supplier<Command> mCommandSupplier;
  Command mCommand;
  public GetAndRunCommand(Supplier<Command> commandSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    mCommandSupplier = commandSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCommand = mCommandSupplier.get();
    addRequirements(mCommand.getRequirements());
    mCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mCommand != null){
      mCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(mCommand != null){
      mCommand.end(mCommand.isFinished());
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mCommand!= null && mCommand.isFinished());
  }
}
