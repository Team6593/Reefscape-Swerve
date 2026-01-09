// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIO extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(ElevatorConstants.mainElevatorID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();
  private SparkMaxConfig m_motorConfig = new SparkMaxConfig();

  private ProfiledPIDController pidController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(62.56, 62.56));
  private ElevatorFeedforward feedForwardController = new ElevatorFeedforward(1, 1.08, 6.87, .11, 0.020);

  public DigitalInput limitSwitch = new DigitalInput(8);

  /** Creates a new ElevatorIO. */
  public ElevatorIO() {
    m_encoder.setPosition(0);

    m_motorConfig.inverted(true);
    m_motorConfig.idleMode(IdleMode.kBrake);
    m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_motorConfig.smartCurrentLimit(60);

    pidController.setTolerance(.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!limitSwitch.get()) {
      resetEncoder();
    }
    SmartDashboard.putNumber("ElevatorIO Pos", m_encoder.getPosition());
    SmartDashboard.putBoolean("ElevatorIO LS", limitSwitch.get());
    SmartDashboard.putBoolean("ElevatorIO At Setpoint", atSetpoint());
    SmartDashboard.putNumber("ElevatorIO DC", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator AA", m_motor.getOutputCurrent());
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public void reachGoal(double goal) {
    pidController.setGoal(goal);
    double pidOutput = pidController.calculate(m_encoder.getPosition(), goal);
    // double feedForwardOutput = feedForwardController.calculate(pidController.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void rawSpeed(double speed) {
    m_motor.set(speed);
  }

  public boolean atSetpoint() {
    return pidController.atGoal();
  }


}
