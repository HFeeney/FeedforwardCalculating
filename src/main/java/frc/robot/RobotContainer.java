// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AngleMotor;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Characterizable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // AngleMotor angleMotor = new AngleMotor();
  Shooter shooter = new Shooter();
  Subsystem testSystem = shooter;

  private Joystick stick = new Joystick(0);
  private JoystickButton a = new JoystickButton(stick, 1);
  private JoystickButton b = new JoystickButton(stick, 2);
  private JoystickButton x = new JoystickButton(stick, 3);
  private JoystickButton y = new JoystickButton(stick, 4);

  private ArrayList<Double> speeds = new ArrayList<>(); 
  private HashMap<Double, Double> voltsToSpeed = new HashMap<>();
  public static double voltage = 1;
  static double phaseDuration = 5;
  public static double rampDuration = 3.0;
  // private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(1.05, 1.537);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.1860, 0.002110);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    testSystem.setDefaultCommand(
      new RunCommand(
        () -> {
          // ((Characterizable) testSystem).setVoltage(stick.getRawAxis(0) * 10);

          double setSpeed = stick.getRawAxis(0) * 6000;
          double ffOutput = ff.calculate(setSpeed);
          SmartDashboard.putNumber("Set Speed", setSpeed);
          SmartDashboard.putNumber("ffOutput", ffOutput);
          ((Characterizable) testSystem).setVoltage(ffOutput);
        }, 
        testSystem
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    a.whenPressed(
      new SequentialCommandGroup(
        new RunCommand(
          () -> {
            ((Characterizable) testSystem).setVoltage(voltage);
          }, 
          testSystem
        ).withTimeout(rampDuration),
        new RunCommand(
          () -> {
            ((Characterizable) testSystem).setVoltage(voltage);
            speeds.add(((Characterizable) testSystem).getVelocity());
          }, 
          testSystem
        ).withTimeout(phaseDuration),
        new InstantCommand(
          () -> {
            ((Characterizable) testSystem).setVoltage(0);
            int numSpeeds = speeds.size();
            double avg = 0.0;
            while (!speeds.isEmpty())
              avg += speeds.remove(0);
            avg = avg / numSpeeds;
            voltsToSpeed.put(voltage, avg);
            SmartDashboard.putNumber("" + voltage, avg);
          }
        )
      )
    );

    b.whenPressed(
      new InstantCommand(
        () -> {voltage+= 0.5;}
      )
    );

    x.whenPressed(
      new InstantCommand(
        () -> {voltage-= 0.5;}
      )
    );

    y.whenPressed(
      new InstantCommand(
        () -> {voltage *= -1;}
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
