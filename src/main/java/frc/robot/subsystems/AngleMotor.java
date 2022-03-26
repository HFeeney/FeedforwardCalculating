// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Characterizable;

public class AngleMotor extends SubsystemBase implements Characterizable{

  private CANSparkMax motor;
  private AnalogInput encoder;
  private AnalogInput a;
  private AnalogInput b;
  private AnalogInput c;
  private double prevPosition;
  private double encoderVelocity = 0;
  private double voltage = 0;
  private double prevVelocity = 0;
  
  private static final double CPR = 4.957;
  
  /** Creates a new AngleMotor. */
  public AngleMotor() {
    motor = new CANSparkMax(4, MotorType.kBrushless);
    encoder = new AnalogInput(1);
    // a = new AnalogInput(1);
    // b = new AnalogInput(2);
    // c = new AnalogInput(3);
    prevPosition = getEncoderPosition();
  }

  public void setVoltage(double v) {
    motor.setVoltage(v);
    voltage = v;
  }

  public double getEncoderPosition() {
    return CPR - encoder.getAverageVoltage();
  }

  // in encoder ticks / second
  public double getVelocity() {
    return encoderVelocity;
  }

  private void updateEncoderVelocity() {
    double shiftedPrevious = prevPosition - getEncoderPosition();
    double positionChange;

    if (shiftedPrevious < 0)
      shiftedPrevious += CPR;
    
    if (shiftedPrevious < CPR / 2.0)
      positionChange = -1 * shiftedPrevious;
    else
      positionChange = CPR - shiftedPrevious;

    encoderVelocity = positionChange / 0.02;
    prevPosition = getEncoderPosition();
  }


  @Override
  public void periodic() {
    updateEncoderVelocity();
    double encoderVelocity = getVelocity();
    
    SmartDashboard.putNumber("Set Voltage", voltage);
    SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
    // SmartDashboard.putNumber("Encoder 1 Position", a.getAverageVoltage());
    // SmartDashboard.putNumber("Encoder 2 Position", b.getAverageVoltage());
    // SmartDashboard.putNumber("Encoder 3 Position", c.getAverageVoltage());
    SmartDashboard.putNumber("Encoder Velocity", getVelocity());
    SmartDashboard.putNumber("Encoder Acceleration", (encoderVelocity - prevVelocity) / 0.02);
    
    prevVelocity = encoderVelocity;
  }
}
