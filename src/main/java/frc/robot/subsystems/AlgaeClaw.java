package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaeClawConstants;
import frc.robot.RobotMap;

public class AlgaeClaw {
    private TalonFX mClawRollerMotor;
    private SparkFlex mClawAngleMotor;

    public AlgaeClaw() {
        mClawAngleMotor = new SparkFlex(0, null)
        AbsoluteEncoderConfig clawEncoderConfig = new AbsoluteEncoderConfig();
        //configure through bore encoder. We will Zero them in rev's hardware client.
        clawEncoderConfig.positionConversionFactor(360);
        clawEncoderConfig.startPulseUs(1.0);
        clawEncoderConfig.endPulseUs(1024.0);
        clawEncoderConfig.inverted(false);

        //configure closed loop control of intake arm
        MAXMotionConfig clawMoveConfig = new MAXMotionConfig();
        clawMoveConfig.allowedClosedLoopError(AlgaeClawConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR);
        clawMoveConfig.maxAcceleration(AlgaeClawConstants.MAX_MOTION_MAX_ACCELERATION);
        clawMoveConfig.maxVelocity(AlgaeClawConstants.MAX_MOTION_MAX_VELOCITY);
        clawMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        ClosedLoopConfig armClosedLoopConfig = new ClosedLoopConfig();
        armClosedLoopConfig.pid(AlgaeClawConstants.MOVING_P_GAIN, AlgaeClawConstants.MOVING_I_GAIN, AlgaeClawConstants.MOVING_D_GAIN, ClosedLoopSlot.kSlot0);
        armClosedLoopConfig.apply(clawMoveConfig);

        clawCon.apply(armEncoderConfig);
        mIntakeConfig.apply(armClosedLoopConfig);
        mIntakeConfig.smartCurrentLimit(getIntakeArmCurrentLimit());
        mIntakeConfig.inverted(isIntakeArmMotorInverted());
        mIntakeConfig.idleMode(IdleMode.kCoast);
        mIntakeArmMotor.configure(mIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //configure intake roller motor
        mClawRollerMotor = new TalonFX(RobotMap.INTAKE_ROLLER_MOTOR_ID);
        TalonFXConfigurator intakeRollerConfigurator = mClawRollerMotor.getConfigurator();
        CurrentLimitsConfigs intakeRollerCurrentConfigs = new CurrentLimitsConfigs();
        intakeRollerCurrentConfigs.withSupplyCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT);
        intakeRollerCurrentConfigs.withStatorCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_STATOR_CURRENT_LIMIT);

        TalonFXConfiguration mIntakeRollerConfig = new TalonFXConfiguration();
        mIntakeRollerConfig.withCurrentLimits(intakeRollerCurrentConfigs);
        MotorOutputConfigs rollerOutputConfig = new MotorOutputConfigs();
        rollerOutputConfig.withInverted(AlgaeClawConstants.IS_INTAKE_ROLLER_INVERTED? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive);
        rollerOutputConfig.withNeutralMode(NeutralModeValue.Coast);
        mIntakeRollerConfig.withMotorOutput(rollerOutputConfig);
        intakeRollerConfigurator.apply(mIntakeRollerConfig);
    }
}
