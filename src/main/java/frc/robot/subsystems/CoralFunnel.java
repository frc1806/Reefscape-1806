// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralFunnelConstants;
import frc.robot.commands.ClawToPosition;
import frc.robot.RobotMap;

public class CoralFunnel extends SubsystemBase {
  //Needs 1 Spark Max (runs a Neo550)
  //Current limit of 20 Amps
  //Needs to be a singleton subsystem (one 1 ever, use GetInstance)
  //Need to be able to run it fowards, run it backwards, and stop it
  //Re-use AlgaeIntake CAN IDs since we won't have a bespoke Algae Intake

  private static final CoralFunnel S_INSTANCE = new CoralFunnel();

  public static CoralFunnel GetInstance(){
      return S_INSTANCE;
  }
  private SparkMax mCoralFunnelRoller;
  
  /** Creates a new CoralFunnel. */
  public CoralFunnel() {
    mCoralFunnelRoller = new SparkMax(RobotMap.CORAL_FUNNEL_ROLLER_ID, MotorType.kBrushless);
    LimitSwitchConfig mCorrelFunneLimitSwitchConfig = new LimitSwitchConfig().forwardLimitSwitchEnabled(false).forwardLimitSwitchType(Type.kNormallyOpen);
    SparkBaseConfig mCorralFunnelRollerConfig = new SparkMaxConfig().smartCurrentLimit(20).apply(mCorrelFunneLimitSwitchConfig).inverted(true);
    mCoralFunnelRoller.configure(mCorralFunnelRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runRollersForward(){
    mCoralFunnelRoller.setVoltage(CoralFunnelConstants.CORAL_FUNNEL_ROLLER_OUT_VOLTAGE);
  }

  public void runRollersBackwards(){
    mCoralFunnelRoller.setVoltage(CoralFunnelConstants.CORAL_FUNNEL_ROLLER_IN_VOLTAGE);
  }

  public void stop(){
    mCoralFunnelRoller.stopMotor();
  }

  public boolean DoesCoralTrayHaveCoral(){
    return mCoralFunnelRoller.getReverseLimitSwitch().isPressed();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  }