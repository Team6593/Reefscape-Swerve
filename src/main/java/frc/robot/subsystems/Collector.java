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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Collector extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless);
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  private SparkMax collectorMotor = new SparkMax(AlgaeConstants.intakeMotorID, MotorType.kBrushless);
  private SparkMaxConfig topConfig = new SparkMaxConfig();
  private SparkClosedLoopController topMotorController = collectorMotor.getClosedLoopController();
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  public DigitalInput limitSwitch = new DigitalInput(1);

  /** Creates a new Algae. */
  public Collector() {
    pivotConfig.idleMode(IdleMode.kBrake);
    topConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.p(.02).i(0).d(0);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    pivotEncoder.setPosition(0);

    collectorMotor.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    pivotController.setReference(40, ControlType.kCurrent);
    //topMotorController.setReference(20, ControlType.kCurrent);
  }

  public void intakeUntilSwitch(double speed) {
    if (limitSwitch.get()) {
      collectorMotor.set(speed);
    } else if (!limitSwitch.get()) {
      collectorMotor.stopMotor();
    }
  }

  public void intakeAlgae(double speed) {
    //collectorMotor.set(speed);
  }

  public void movePivot(double speed) {
    pivotMotor.set(speed);
  }

  public void pivotToSetpoint() {
    pivotController.setReference(15.8, ControlType.kPosition);
  }

  public void pivotBack() {
    pivotController.setReference(0, ControlType.kPosition);
  }

  // public void ToSetpoint(double setpoint) {
  //   pivotController.setReference(setpoint, ControlType.kPosition);
  // }

  public void stop() {
    pivotMotor.set(0);
    collectorMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("switch", limitSwitch.get());
    SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
