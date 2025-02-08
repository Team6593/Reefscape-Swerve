// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless);
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  private SparkMax topMotor = new SparkMax(AlgaeConstants.topMotorID, MotorType.kBrushless);
  private SparkMaxConfig topConfig = new SparkMaxConfig();
  private SparkClosedLoopController topMotorController = topMotor.getClosedLoopController();
  private RelativeEncoder topEncoder = topMotor.getEncoder();

  private SparkMax bottomMotor = new SparkMax(AlgaeConstants.bottomMotorID, MotorType.kBrushless);
  private SparkClosedLoopController bottomMotorController = bottomMotor.getClosedLoopController();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private DigitalInput beamBrake = new DigitalInput(AlgaeConstants.beamBrakeID);

  /** Creates a new Algae. */
  public Algae() {
    pivotConfig.idleMode(IdleMode.kBrake);
    //pivotConfig.closedLoop.p(.1).i(0).d(0);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    topConfig.inverted(true);
    topMotor.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    pivotController.setReference(20, ControlType.kCurrent);
    topMotorController.setReference(20, ControlType.kCurrent);
    bottomMotorController.setReference(20, ControlType.kCurrent);
  }

  public void IntakeAlgae(double speed) {
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  public void MovePivot(double speed) {
    pivotMotor.set(speed);
  }

  // public void ToSetpoint(double setpoint) {
  //   pivotController.setReference(setpoint, ControlType.kPosition);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
