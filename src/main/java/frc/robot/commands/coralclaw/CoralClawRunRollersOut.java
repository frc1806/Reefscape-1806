// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.coralclaw;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralClaw;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralClawRunRollersOut extends Command {
//   /** Creates a new CoralClawRunRollersOut. */
//   public CoralClawRunRollersOut() {
//     addRequirements(CoralClaw.GetInstance());
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     CoralClaw.GetInstance().runRollersOut();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     CoralClaw.GetInstance().runRollersOut();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     CoralClaw.GetInstance().stopRollers();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
