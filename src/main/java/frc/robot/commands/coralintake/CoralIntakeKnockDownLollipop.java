// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.coralintake;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.intakes.CoralIntake;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralIntakeKnockDownLollipop extends Command {
//   /** Creates a new CoralIntakeKnockDownLollipop. */
//   public CoralIntakeKnockDownLollipop() {
//     addRequirements(CoralIntake.GetInstance());
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     CoralIntake.GetInstance().goToArbitraryPosition(60.0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return CoralIntake.GetInstance().isAtArbitraryPosition(60.0);
//   }
// }
