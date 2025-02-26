// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class KrakenElevator extends SubsystemBase {

  private TalonFX rightMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID);
  private TalonFX leftMotor = new TalonFX(ElevatorConstants.leftElevatorMotorID);

  /** Creates a new KrakenElevator. */
  public KrakenElevator() {
  
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    leftMotor.setNeutralMode(NeutralModeValue.Brake);

    // Slot 0 Gains
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = .5;

    // Motion Magic Settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1000;
    motionMagicConfigs.MotionMagicAcceleration = 1500;

    rightMotor.getConfigurator().apply(talonFXConfigs);
    leftMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void elevate(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  /**
   * Moves the elevator to a setpoint via the Kraken x60s.
   * @param setpoint - Rotations?
   */
  public void elevateToSetpoint(double setpoint) {
    // account for gravity
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // setpoints
    rightMotor.setControl(m_request.withPosition(setpoint));
    leftMotor.setControl(m_request.withPosition(setpoint));
  }

  public void stop() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }

  public double getRightReading() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  public double getLeftReading() {
    return leftMotor.getPosition().getValueAsDouble();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ele RMotor Pos", getRightReading());
    SmartDashboard.putNumber("Ele LMotor Pos", getLeftReading());
    SmartDashboard.putNumber("Ele RMotor Vel", rightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Ele LMotor Vel", leftMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Ele RMotor DCycle", rightMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Ele LMotor DCycle", leftMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Ele RMotor Temp", rightMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Ele LMotor Temp", leftMotor.getDeviceTemp().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
