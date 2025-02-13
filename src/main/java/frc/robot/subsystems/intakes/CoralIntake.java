package frc.robot.subsystems.intakes;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;

public class CoralIntake extends MotorizedArmIntake{

    private static final CoralIntake S_INSTANCE = new CoralIntake();

    public static CoralIntake GetInstance(){
        return S_INSTANCE;
    }

    private CoralIntake(){
        super();
    }

    @Override
    protected boolean isIntakeEnabled() {
        return true;
    }

    @Override
    protected String getIntakeName() {
        return "CoralIntake";
    }

    @Override
    protected int getIntakeRollerMotorId() {
        return RobotMap.CORAL_INTAKE_ROLLER_ID;
    }

    @Override
    protected boolean isIntakeRollerInverted() {
        return false;
    }

    @Override
    protected double getIntakeRollerStatorCurrentLimit() {
        return 120.0;
    }

    @Override
    protected double getIntakeRollerSupplyCurrentLimit() {
        return 50.0;
    }

    @Override
    protected int getIntakeArmMotorId() {
        return RobotMap.CORAL_INTAKE_ARM_ID;
    }

    @Override
    protected boolean isIntakeArmMotorInverted() {
        return false;
    }

    @Override
    protected boolean isIntakeArmEncoderInverted() {
        return false;
    }

    @Override
    protected int getIntakeArmCurrentLimit() {
        return 20;
    }

    @Override
    protected double getMovingPGain() {
        return 1.0/30.0;
    }

    @Override
    protected double getMovingIGain() {
        return 0.0;
    }

    @Override
    protected double getMovingDGain() {
        return 0.0;
    }

    @Override
    protected double getMaxMotionMaxVelocity() {
        return 180.0;
    }

    @Override
    protected double getMaxMotionMaxAcceleration() {
        return 360.0;
    }

    @Override
    protected double getMaximumAllowedClosedLoopError() {
        return 1.0;
    }

    @Override
    protected double getHoldingPGain() {
        return 1.0/90.0;
    }

    @Override
    protected double getHoldingIGain() {
        return 0.0;
    }

    @Override
    protected double getHoldingDGain() {
        return 0.0;
    }

    @Override
    protected double getIntakeArmAngleAtRest() {
        return -2.0;
    }

    @Override
    protected double getIntakeArmAngleAtExtension() {
        return 90.0;
    }

    @Override
    protected double getIntakeRollerSpeedIntaking() {
        return 12.0;
    }

    @Override
    protected double getIntakeRollerSpeedSpitOut() {
        return -12.0;
    }

    @Override
    protected double getAcceptableHoldInAngleDeviation() {
        return 5.0;
    }

    @Override
    protected double getAcceptableHoldingOutAngleDeviation() {
        return 15.0;
    }

    @Override
    protected double getMinimumEverReasonableAngle() {
        return -5.0;
    }

    @Override
    protected double getMaximumEverReasonableAngle() {
        return 110.0;
    }

    @Override
    protected double getArmGearRatio() {
        return 5 * 5 * (30.0/18.0);
    }

    @Override
    protected double getArmCenterOfGravityDistance() {
        return Units.inchesToMeters(Math.hypot(6.021, 2.536));
    }

    @Override
    protected double getArmMass() {
        return Units.lbsToKilograms(4.544);
    }

}
