// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.mainElevatorID, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();
  
  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  
  private RelativeEncoder elevatorEncoding = elevatorMotor.getEncoder();

  private DutyCycleEncoder bore = new DutyCycleEncoder(0);
  //private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.mainElevatorID, MotorType.kBrushless).getAlternateEncoder(bore);

  public DigitalInput limitSwitch = new DigitalInput(8);

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorConfig.inverted(false).idleMode(IdleMode.kBrake);
  
    // elevatorConfig.closedLoop.maxMotion
    //   .maxVelocity(4000)
    //   .maxAcceleration(6000);
    
    elevatorConfig.closedLoop
    .p(.5)
    .i(0)
    .d(0)
    .outputRange(-1, 1);

    elevatorMotor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    

    elevatorController.setReference(40, ControlType.kCurrent);

    elevatorEncoding.setPosition(0);

  }

  @Override
  public void periodic() {

    if(!limitSwitch.get()) {
      resetEncoder();
    }

    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right Motor Temp", elevatorMotor.getMotorTemperature());
    SmartDashboard.putNumber("Left Motor Temperature", elevatorMotor.getMotorTemperature());
    SmartDashboard.putBoolean("Elevator Switch", limitSwitch.get());
    SmartDashboard.putNumber("Elevator Encoder Position", elevatorEncoding.getPosition());
    //SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Motor Output", elevatorMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Left Motor Output", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("El Right Velocity", elevatorEncoding.getVelocity());
    //SmartDashboard.putNumber("El Left Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("El Right Speed", elevatorMotor.getAppliedOutput());
    //SmartDashboard.putNumber("EL Left Speed", leftMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Bore Encoder", getBoreEncoder());
  }

  /**
   * Returns the encoder reading from the right motor on the elevator.
   * @return Right encoder position
   */
  public double getRightEncoderReading() {
    return Math.floor(elevatorEncoding.getPosition());
  }

  /**
   * Moves the elevator to a specified encoder position.
   * @param setpoint - Desired encoder position.
   */
  public void goToSetpoint(double setpoint) {
    //rightController.setReference(setpoint, ControlType.kPosition);
    elevatorController.setReference(setpoint, ControlType.kPosition);
    //leftController.setReference(setpoint, ControlType.kPosition);
  }

  /**
   * Changes the elevators motors to brake mode.
   */
  public void changeToBrakeMode() {
    elevatorConfig.idleMode(IdleMode.kBrake);

    elevatorMotor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Elevates the elevator.
   * @param speed - Positive = Up, Negative = Down
   */
  public void elevate(double speed) {
    elevatorMotor.set(speed);
  }

  /**
   * Stops the elevator.
   */
  public void stop() {
    elevatorMotor.set(0);
    //leftMotor.set(0);
  }

  public void resetEncoder() {
    elevatorEncoding.setPosition(0);
  }

  public double getBoreEncoder() {
    return bore.get();
  }

}