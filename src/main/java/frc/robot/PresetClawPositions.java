// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum PresetClawPositions {
    kHome(6.75, 0.0, 0.0),
    kAlgaeNetForward(65.0, 0.0, 135.0),
    kAlgaeNetBackward(65.0, 0.0, 45.0),
    kAlgaeProcessForward(45.0, 0.0, 200.0),
    kAlgaeProcessBackward(0, 0.0, 10.0),
    kAlgaeL3(43.7, 0.0, 200.0),
    kAlgaeL2(35.8, 0.0, 200.0),
    kCoralL4(65.0, 120.0, 0.0),
    kCoralL3(50.0, 150.0, 0.0),
    kCoralL2(34.0, 150.0, 0.0),
    kCoralL1(25, 180.0, 0.0),
    kClawMotionTest(50.0, 0.0, 0.0),

    kClimbPart1(42.0, 0.0, 180.0),
    kClimbPart2(7.5, 0.0, 80.0);
    
    

    private double mElevatorHeight;
    private double mCoralClawAngle;
    private double mAlgaeClawAngle;

    /**
     * Make a preset combination of elevator height, coral claw angle, and algae claw angle.
     * @param elevatorHeight elevator height to the bottom of the carriage in inches
     * @param coralClawAngle coral claw angle in degrees. If you are staring at the claw's axle, its home position (0 degrees) is it pointing straight left, and counterclockwise is positive.
     * @param algaeClawAngle algae claw angle in degrees. If you ar estaring at the claw's axle, its home position (0 degrees) is it pointing straight right, and counterclockwise is positive.
     */
    private PresetClawPositions(double elevatorHeight, double coralClawAngle, double algaeClawAngle){
        mElevatorHeight = elevatorHeight;
        mCoralClawAngle = coralClawAngle;
        mAlgaeClawAngle = algaeClawAngle;
    }

    /**
     * Get elevator carriage bottom height in inches from the floor.
     * @return
     */
    public double getElevatorHeight(){
        return mElevatorHeight;
    }

    /**
     * Get coral claw angle in degrees 
     * @return
     */
    public double getCoralClawAngle(){
        return mCoralClawAngle;
    }

    /**
     * Get algae claw angle in degrees
     * @return
     */
    public double getAlgaeClawAngle(){
        return mAlgaeClawAngle;
    }

}
