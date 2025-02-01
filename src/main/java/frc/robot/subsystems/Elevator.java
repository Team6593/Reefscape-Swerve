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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private SparkMax rightMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);
  private SparkMax leftMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
  private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
  private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  
  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  /** Creates a new Elevator. */
  public Elevator() {
    rightConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftConfig.inverted(false).idleMode(IdleMode.kBrake);
    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightController.setReference(20, ControlType.kCurrent);
    leftController.setReference(20, ControlType.kCurrent);
    // rightConfig.closedLoop
    //   .p(.1)
    //   .i(0)
    //   .d(0)
    //   .outputRange(-.3, .3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
  }

  public void changeToCoastMode() {
    rightConfig.idleMode(IdleMode.kCoast);
    leftConfig.idleMode(IdleMode.kCoast);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void changeToBrakeMode() {
    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void up(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  public void down(double speed) {
    rightMotor.set(-speed);
    leftMotor.set(-speed);
  }

  public void stop() {
    rightMotor.set(0);
    leftMotor.set(0);
  }

  public void moveToSetpoint(double speed, double setpoint) {
    if (setpoint > leftEncoder.getPosition()) {
      up(speed);
    } else if (setpoint == leftEncoder.getPosition()) {
      stop();
    } else if (setpoint < leftEncoder.getPosition()) {
      down(speed);
    }
  }

}
