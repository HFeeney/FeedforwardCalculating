// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final double METERS_PER_INCH = 0.0254;

    public static class Shooter {
        public static final int FORWARD_CHANNEL_PORT = 4;
        public static final int REVERSE_CHANNEL_PORT = 5;
        public static final Value RAISED_VALUE = Value.kReverse;
        public static final Value LOWERED_VALUE = Value.kForward; // TODO: find correct directions
        public static final int MOTOR_1_PORT = 6; 
        public static final int MOTOR_2_PORT = 7;
        public static final boolean MOTOR_1_INVERTED = false;
        public static final boolean MOTOR_2_INVERTED = true;
        public static final double RAMP_RATE = 1.3;
        public static final double GEAR_RATIO = 1.5; // 1.5 motor rotaion for every motor
        public static final double UPPER_HUB_SPEED_PERCENTAGE = 1.0;// 0.75;
        public static final double UPPER_HUB_RPM_THRESHOLD = 4000;
        public static final double LOWER_HUB_SPEED_PERCENTAGE = 0.4;// 0.75;
        public static final double LOWER_HUB_RPM_THRESHOLD = 1500;
    }

    public static class Control {
        public static class Driver {
            public static final int PORT = 0;
            public static final double LEFT_X_DEADBAND = 0.1;
            public static final double LEFT_Y_DEADBAND = 0.1;
            public static final double RIGHT_X_DEADBAND = 0.1;
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
        }

        public static class Manipulator {
            public static final int PORT = 1;
        }
    }
}
