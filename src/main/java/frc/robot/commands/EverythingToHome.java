// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.PresetClawPositions;
//import frc.robot.commands.coralintake.CoralIntakeRetract;
import frc.robot.commands.coralintake.CoralIntakeRetract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EverythingToHome extends ParallelCommandGroup {
  /** Creates a new EverythingToHome. */
  public EverythingToHome() {
    addCommands(
      new ClawToPosition(PresetClawPositions.kHome),
      new CoralIntakeRetract() 
       );
  }
}
