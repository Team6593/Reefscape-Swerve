// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private TalonFX pivotMotor = new TalonFX(ClimberConstants.climberID);
  private TalonFX winchMotor = new TalonFX(ClimberConstants.winchID);

  private PositionVoltage positionVoltage = new PositionVoltage(0);

  /** Creates a new Climber. */
  public Climber() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = .25;
    slot0Configs.kV = .12;
    slot0Configs.kP = .1;
    slot0Configs.kI = 0;
    slot0Configs.kD = .1;
    pivotMotor.getConfigurator().apply(slot0Configs);

    final TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(1, 4)
    );
    TrapezoidProfile.State goal = new TrapezoidProfile.State(4, 0);
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    final PositionVoltage request = new PositionVoltage(0).withSlot(0);

    setpoint = profile.calculate(.020, setpoint, goal);

    request.Position = setpoint.position;
    request.Velocity = setpoint.velocity;
    pivotMotor.setControl(request);

  }

  public void pivot(double speed) {
    pivotMotor.set(.27 * -speed);
    winchMotor.set(speed);
  }

  // public void pivotToSetpoint(double setpoint) {
  //   pivotMotor.setControl(positionVoltage.withPosition(setpoint));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
