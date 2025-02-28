// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PresetClawPositions;
import frc.robot.commands.algaeclaw.AlgaeClawToAngle;
import frc.robot.commands.elevator.ElevatorMoveSequence;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FinishClimbSequence extends SequentialCommandGroup {
  /** Creates a new FinishClimbSequence. */
  public FinishClimbSequence() {
    new ParallelCommandGroup(new ElevatorMoveSequence(PresetClawPositions.kClimbPart2.getElevatorHeight()), new AlgaeClawToAngle(PresetClawPositions.kClimbPart2.getAlgaeClawAngle()));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
