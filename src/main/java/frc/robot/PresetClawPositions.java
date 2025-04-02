// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

/** Add your docs here. */
public enum PresetClawPositions {
    kHome(29.0, 5.0),
    kFeeder(32.0, 5.0),
    kCoralTransfer(22.0, 0.0),
    kAlgaeHold(12.0, 150.0),
    kAlgaeIntake(22.0, 60.0),
    kAlgaeNetForward(45.0, 160.0),
    kAlgaeNetBackward(45.0, 200.0),
    kAlgaeProcessForward(25.0, 60.0),
    kAlgaeL3(45.0, 65.0),
    kAlgaeL2(36.0, 65.0),
    kCoralL4(42.2, 152.0, 123.0),
    kCoralL3(21.5, 147.0, 123.0),
    kCoralL2(15.5, 128.0, 100.0),
    kCoralL1(40, 40.0),
    kClawMotionTest(30.0, 0.0),

    kClimbPart1(35.0, 0.0),
    kClimbPart2(30.5, 0.0);
    
    

    private double mElevatorHeight;
    private double mTheClawAngle;
    private Optional<Double> mScoringAngle;

    /**
     * Make a preset combination of elevator height, coral claw angle, and algae claw angle.
     * @param elevatorHeight elevator height to the bottom of the carriage in inches
     * @param TheClawAngle algae claw angle in degrees. If you ar estaring at the claw's axle, its home position (0 degrees) is it pointing straight down, and upwards is positive.
     */
    private PresetClawPositions(double elevatorHeight, double theClawAngle){
        mElevatorHeight = elevatorHeight;
        mTheClawAngle = theClawAngle;
        mScoringAngle = Optional.empty();
    }
    
    /**
     * Make a preset combination of elevator height, coral claw angle, and algae claw angle.
     * @param elevatorHeight elevator height to the bottom of the carriage in inches
     * @param TheClawAngle algae claw angle in degrees. If you ar estaring at the claw's axle, its home position (0 degrees) is it pointing straight down, and upwards is positive.
     * @param scoringAngle algae claw angle when scoring at this position
     */
    private PresetClawPositions(double elevatorHeight, double theClawAngle, double scoringAngle)
    {
        mElevatorHeight = elevatorHeight;
        mTheClawAngle = theClawAngle;
        mScoringAngle = Optional.of(scoringAngle);
    }

    /**
     * Get elevator carriage bottom height in inches from the floor.
     * @return
     */
    public double getElevatorHeight(){
        return mElevatorHeight;
    }

    /**
     * Get the claw angle in degrees
     * @return
     */
    public double getTheClawAngle(){
        return mTheClawAngle;
    }
    
    /**
     * If this preset has a scoring angle, return it, if not returns the preset angle so no fishy things happen.
     * @return
     */
    public double getClawScoringAngle(){
        if(mScoringAngle.isPresent()){
            return mScoringAngle.get();
        }
        return mTheClawAngle;
    }

}
