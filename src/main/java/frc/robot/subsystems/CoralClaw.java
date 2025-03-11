// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.AbsoluteEncoderConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.MAXMotionConfig;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AlgaeClawConstants;
// import frc.robot.Constants.CoralClawConstants;
// import frc.robot.RobotContainer;
// import frc.robot.RobotMap;

// public class CoralClaw extends SubsystemBase {
//   private static CoralClaw S_INSTANCE = new CoralClaw();

//   public static CoralClaw GetInstance(){
//     return S_INSTANCE;
//   }

//   SparkFlex mAngleMotor;
//   SparkFlex mOpenClaw;
//   TalonFX mRollerMotor;
//   double mTargetAngle;
//   /** Creates a new CoralClaw. */
//   private CoralClaw() {
//     mAngleMotor = new SparkFlex(RobotMap.CORAL_CLAW_ANGLE_ID, MotorType.kBrushless);
//     SparkBaseConfig clawAngleConfig = new SparkFlexConfig();
//     AbsoluteEncoderConfig clawEncoderConfig = new AbsoluteEncoderConfig();
//     //configure through bore encoder. We will Zero them in rev's hardware client.
//     clawEncoderConfig.positionConversionFactor(360);
//     clawEncoderConfig.startPulseUs(1.0);
//     clawEncoderConfig.endPulseUs(1024.0);
//     clawEncoderConfig.inverted(false);

//     //configure closed loop control of intake arm
//     MAXMotionConfig clawMoveConfig = new MAXMotionConfig();
//     clawMoveConfig.allowedClosedLoopError(CoralClawConstants.ClawRotationConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR);
//     clawMoveConfig.maxAcceleration(CoralClawConstants.ClawRotationConstants.MAX_MOTION_MAX_ACCELERATION);
//     clawMoveConfig.maxVelocity(CoralClawConstants.ClawRotationConstants.MAX_MOTION_MAX_VELOCITY);
//     clawMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

//     ClosedLoopConfig armClosedLoopConfig = new ClosedLoopConfig();
//     armClosedLoopConfig.pid(CoralClawConstants.ClawRotationConstants.MOVING_P_GAIN, CoralClawConstants.ClawRotationConstants.MOVING_I_GAIN, CoralClawConstants.ClawRotationConstants.MOVING_D_GAIN, ClosedLoopSlot.kSlot0);
//     armClosedLoopConfig.apply(clawMoveConfig);

//     clawAngleConfig.apply(clawEncoderConfig);
//     clawAngleConfig.apply(armClosedLoopConfig);
//     clawAngleConfig.smartCurrentLimit(CoralClawConstants.ClawRotationConstants.CURRENT_LIMIT);
//     clawAngleConfig.inverted(AlgaeClawConstants.CLAW_INTAKE_ARM_INVERTED);
//     clawAngleConfig.idleMode(IdleMode.kBrake);
//     mAngleMotor.configure(clawAngleConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     SparkBaseConfig clawOpenCloseConfig = new SparkFlexConfig();
//     AbsoluteEncoderConfig clawOpenCloseEncoderConfig = new AbsoluteEncoderConfig();
//     //configure through bore encoder. We will Zero them in rev's hardware client.
//     clawOpenCloseEncoderConfig.positionConversionFactor(360);
//     clawOpenCloseEncoderConfig.startPulseUs(1.0);
//     clawOpenCloseEncoderConfig.endPulseUs(1024.0);
//     clawOpenCloseEncoderConfig.inverted(false);

//     //configure closed loop control of intake arm
//     MAXMotionConfig openCloseMoveConfig = new MAXMotionConfig();
//     openCloseMoveConfig.allowedClosedLoopError(CoralClawConstants.ClawRotationConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR);
//     openCloseMoveConfig.maxAcceleration(CoralClawConstants.ClawRotationConstants.MAX_MOTION_MAX_ACCELERATION);
//     openCloseMoveConfig.maxVelocity(CoralClawConstants.ClawRotationConstants.MAX_MOTION_MAX_VELOCITY);
//     openCloseMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

//     ClosedLoopConfig openCloseClosedLoopConfig = new ClosedLoopConfig();
//     openCloseClosedLoopConfig.pid(CoralClawConstants.ClawRotationConstants.MOVING_P_GAIN, CoralClawConstants.ClawRotationConstants.MOVING_I_GAIN, CoralClawConstants.ClawRotationConstants.MOVING_D_GAIN, ClosedLoopSlot.kSlot0);
//     openCloseClosedLoopConfig.apply(openCloseMoveConfig);
//     openCloseClosedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

//     clawOpenCloseConfig.apply(clawOpenCloseEncoderConfig);
//     clawOpenCloseConfig.apply(openCloseClosedLoopConfig);
//     clawOpenCloseConfig.smartCurrentLimit(CoralClawConstants.ClawRotationConstants.CURRENT_LIMIT);
//     clawOpenCloseConfig.inverted(AlgaeClawConstants.CLAW_INTAKE_ARM_INVERTED);
//     clawOpenCloseConfig.idleMode(IdleMode.kBrake);

//     mOpenClaw = new SparkFlex(RobotMap.CORAL_CLAW_OPEN_CLOSE_ID, MotorType.kBrushless);
//     mOpenClaw.configure(clawOpenCloseConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     mRollerMotor = new TalonFX(RobotMap.ALGAE_CLAW_ROLLER_MOTOR_ID);
//      TalonFXConfigurator intakeRollerConfigurator = mRollerMotor.getConfigurator();
//         CurrentLimitsConfigs intakeRollerCurrentConfigs = new CurrentLimitsConfigs();
//         intakeRollerCurrentConfigs.withSupplyCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT);
//         intakeRollerCurrentConfigs.withStatorCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_STATOR_CURRENT_LIMIT);

//         TalonFXConfiguration mIntakeRollerConfig = new TalonFXConfiguration();
//         mIntakeRollerConfig.withCurrentLimits(intakeRollerCurrentConfigs);
//         MotorOutputConfigs rollerOutputConfig = new MotorOutputConfigs();
//         rollerOutputConfig.withInverted(AlgaeClawConstants.IS_INTAKE_ROLLER_INVERTED? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive);
//         rollerOutputConfig.withNeutralMode(NeutralModeValue.Coast);
//         mIntakeRollerConfig.withMotorOutput(rollerOutputConfig);
//         intakeRollerConfigurator.apply(mIntakeRollerConfig);

//         mTargetAngle = 0;
//   }

//   public void rotateToAngle(double angle){
//     mTargetAngle = angle;
//     mAngleMotor.getClosedLoopController().setReference(angle, ControlType.kMAXMotionPositionControl);
//   }

//   public void openClaw(){
//     mOpenClaw.getClosedLoopController().setReference(CoralClawConstants.ClawOpenCloseConstants.OPEN_ANGLE, ControlType.kMAXMotionPositionControl);
//   }

//   public boolean isClawOpen(){
//     return Math.abs(mOpenClaw.getAbsoluteEncoder().getPosition() - CoralClawConstants.ClawOpenCloseConstants.OPEN_ANGLE) < CoralClawConstants.ClawOpenCloseConstants.ANGLE_TOLERANCE;
//   }

//   public void closeClaw(){
//     mOpenClaw.getClosedLoopController().setReference(CoralClawConstants.ClawOpenCloseConstants.CLOSE_ANGLE, ControlType.kMAXMotionPositionControl);
//   }

//   public boolean isClawClosed(){
//     return Math.abs(mOpenClaw.getAbsoluteEncoder().getPosition() - CoralClawConstants.ClawOpenCloseConstants.CLOSE_ANGLE) < CoralClawConstants.ClawOpenCloseConstants.ANGLE_TOLERANCE;
//   }

//   public void runRollersIn(){
//     mRollerMotor.setControl(new VoltageOut(CoralClawConstants.ROLLER_IN_VOLTAGE));
//   }

//   public void stopRollers(){
//     mRollerMotor.stopMotor();
//   }

//   public void runRollersOut(){
//     mRollerMotor.setControl(new VoltageOut(CoralClawConstants.ROLLER_OUT_VOLTAGE));
//   }

//   public double getRotationAngle(){
//     return mAngleMotor.getAbsoluteEncoder().getPosition();
//   }

//   public boolean isClawAtAngle(){
//     return Math.abs(getRotationAngle() - mTargetAngle) < CoralClawConstants.ClawRotationConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR;
//   }

//   public boolean hasGamePiece(){
//     return RobotContainer.S_CARRIAGE_CANDI.getS1Closed().getValue();
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
