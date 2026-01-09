// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIO extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(ElevatorConstants.mainElevatorID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();

  private ProfiledPIDController pidController = new ProfiledPIDController(8.81, 0, 1.48, new TrapezoidProfile.Constraints(2.45, 2.45));
  private ElevatorFeedforward feedForwardController = new ElevatorFeedforward(0, 1.94, 3.82, 2, 0.020);

  public DigitalInput limitSwitch = new DigitalInput(8);

  /** Creates a new ElevatorIO. */
  public ElevatorIO() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!limitSwitch.get()) {
      resetEncoder();    
    }
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public void reachGoal(double goal) {
    pidController.setGoal(goal);
    double pidOutput = pidController.calculate(m_encoder.getPosition());
    double feedForwardOutput = feedForwardController.calculate(pidController.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedForwardOutput);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void rawSpeed(double speed) {
    m_motor.set(speed);
  }


}
