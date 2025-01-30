// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedElevator extends SubsystemBase {

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearBox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(5, 0, 0, 
    new TrapezoidProfile.Constraints(2.45, 2.45));

  ElevatorFeedforward m_feedForward =
    new ElevatorFeedforward(0, 0.762, 0.762, 0);
  
  private final Encoder m_encoder = new Encoder(0, 1);
  private final PWMSparkMax m_motor = new PWMSparkMax(0);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_ElevatorSim =
    new ElevatorSim(LinearSystemId.createElevatorSystem(m_elevatorGearBox, 4.0, Units.inchesToMeters(2), 10), 
    m_elevatorGearBox, 0.0, 1.25, true, 0, 2);
  
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d = 
    m_mech2dRoot.append(new MechanismLigament2d("Elevator", m_ElevatorSim.getPositionMeters(), 90));

  // Create a real encoder object on DIO 2,3
  Encoder encoder = new Encoder(2, 3);
  // Create a sim controller for the encoder
  EncoderSim simEncoder = new EncoderSim(encoder);

  /** Creates a new SimulatedElevator. */
  public SimulatedElevator() {
    m_encoderSim.setDistancePerPulse(2.0 * Math.PI * Units.inchesToMeters(2) / 4096);

    // Publish Mechanism2d to Smartboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run

    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltage)
    m_ElevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_ElevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistancePerPulse(m_ElevatorSim.getPositionMeters());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSim.getCurrentDrawAmps())
    );

    simEncoder.setCount(100);
    encoder.get(); // 100
    simEncoder.getCount(); // 100
  }

  /**
   * Run control loop to reach and maintain goal.
   * 
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedForwardOutput = m_feedForward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedForwardOutput); 
  }

  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
