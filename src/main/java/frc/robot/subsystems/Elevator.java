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
    leftConfig.follow(ElevatorConstants.rightElevatorMotorID);

    rightConfig.closedLoop
      .p(.1)
      .i(0)
      .d(0)
      .outputRange(-.2, .2);
    // leftConfig.closedLoop
    //   .p(.1)
    //   .i(0)
    //   .d(0);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightController.setReference(40, ControlType.kCurrent);
    leftController.setReference(40, ControlType.kCurrent);

    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);

    changeToCoastMode();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
    //SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
  }

  /**
   * Returns the encoder reading from the right motor on the elevator.
   * @return Right encoder position
   */
  public double getRightEncoderReading() {
    return Math.round(rightEncoder.getPosition());
  }

  /**
   * Moves the elevator to a specified encoder position.
   * @param setpoint - Desired encoder position.
   */
  public void goToSetpoint(double setpoint) {
    rightController.setReference(setpoint, ControlType.kPosition);
  }

  /**
   * Changes the elevators motors to coast mode.
   */
  public void changeToCoastMode() {
    rightConfig.idleMode(IdleMode.kCoast);
    leftConfig.idleMode(IdleMode.kCoast);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Changes the elevators motors to brake mode.
   */
  public void changeToBrakeMode() {
    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Elevates the elevator.
   * @param speed - Positive = Up, Negative = Down
   */
  public void elevate(double speed) {
    rightMotor.set(speed);
    //leftMotor.set(speed);
  }

  /**
   * Stops the elevator.
   */
  public void stop() {
    rightMotor.set(0);
    //leftMotor.set(0);
  }

}
