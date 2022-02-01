// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleMotor extends SubsystemBase {

  private WPI_TalonSRX motor;
  private AnalogInput encoder;
  private double prevPosition;
  private double encoderVelocity = 0;
  private double voltage = 0;
  
  private static final double CPR = 4.927;
  
  /** Creates a new AngleMotor. */
  public AngleMotor() {
    motor = new WPI_TalonSRX(0);
    encoder = new AnalogInput(3);
    prevPosition = getEncoderPosition();
  }

  public void setVoltage(double v) {
    motor.setVoltage(v);
    voltage = v;
  }

  public double getEncoderPosition() {
    return encoder.getAverageVoltage();
  }

  // in encoder ticks / second
  public double getEncoderVelocity() {
    return encoderVelocity;
  }


  @Override
  public void periodic() {
    double positionDifference = getEncoderPosition() - prevPosition >= 0.0 ? getEncoderPosition() - prevPosition : getEncoderPosition() + CPR - prevPosition;
    encoderVelocity = positionDifference / 0.02; 
    
    SmartDashboard.putNumber("Set Voltage", voltage);
    SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
    SmartDashboard.putNumber("Encoder Velocity", getEncoderVelocity());
    
    prevPosition = getEncoderPosition();
  }
}
