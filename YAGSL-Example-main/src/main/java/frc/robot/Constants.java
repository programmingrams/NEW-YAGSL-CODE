// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

    public static final class ElevatorConstants {
        public static final int LEADER_MOTOR_ID = 10;
        public static final int FOLLOWING_MOTOR_ID = 11;
        public static final int LOWER_LIMIT_ID = 0;
        public static final int LOWER_LIMIT_ID_2 = 0;

        public static final double INITIAL_HEIGHT_ROT = 0;

        public static final double STOW_ROT = 0.1;
        public static final double L1_ROT = 0.33333;
        public static final double L2_ROT = 23.2141;
        public static final double L3_ROT = 53.857;
        public static final double L4_ROT = 87.26568603515625;
        public static final double IN_ROT = 10.42862;

        public static final double P_UP = 0.1;
        public static final double I_UP = 0.0;
        public static final double D_UP = 0;

        public static final double P_DOWN = 0.1;
        public static final double I_DOWN = 0.0;
        public static final double D_DOWN = 0.0;

        public static final double MAX_VELOCITY_UP = 9000;
        public static final double MAX_ACCELERATION_UP = 9000;
        public static final double ALLOWED_ERROR_UP = 0.25;

        public static final double MAX_VELOCITY_DOWN = 3500;
        public static final double MAX_ACCELERATION_DOWN = 6000;
        public static final double ALLOWED_ERROR_DOWN = 0.1;
            
        public static final boolean LEADER_INVERTED = false;
        public static final boolean FOLLOWER_INVERTED_FROM_LEADER = true;
        public static final int CURRENT_LIMIT = 30;
        public static final double RAMP_RATE = .01;
        public static final double OUTPUT_MINIMUM = -1.0;
        public static final double OUTPUT_MAXIMUM = 1.0;
        
        public static final SparkMaxConfig MOTOR_CONFIG_COAST = new SparkMaxConfig() {{
            idleMode(IdleMode.kCoast);
            smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT, ElevatorConstants.CURRENT_LIMIT*2);
            inverted(ElevatorConstants.LEADER_INVERTED);
            openLoopRampRate(ElevatorConstants.RAMP_RATE);
            closedLoop.outputRange(-1, OUTPUT_MAXIMUM, ClosedLoopSlot.kSlot0) // kslot 0 is up
                .p(ElevatorConstants.P_UP, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.I_UP, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.D_UP, ClosedLoopSlot.kSlot0);
            closedLoop.maxMotion.maxVelocity(MAX_VELOCITY_UP, ClosedLoopSlot.kSlot0)
                .maxAcceleration(MAX_ACCELERATION_UP, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(ALLOWED_ERROR_UP, ClosedLoopSlot.kSlot0);

            closedLoop.outputRange(OUTPUT_MINIMUM, 1, ClosedLoopSlot.kSlot1) // kslot 1 is down
                .p(ElevatorConstants.P_DOWN, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.I_DOWN, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.D_DOWN, ClosedLoopSlot.kSlot1);
            closedLoop.maxMotion.maxVelocity(MAX_VELOCITY_DOWN, ClosedLoopSlot.kSlot1)
                .maxAcceleration(MAX_ACCELERATION_DOWN, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(ALLOWED_ERROR_DOWN, ClosedLoopSlot.kSlot1);
        }};

        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig() {{
            idleMode(IdleMode.kBrake);
            smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT, ElevatorConstants.CURRENT_LIMIT*2);
            inverted(ElevatorConstants.LEADER_INVERTED);
            openLoopRampRate(ElevatorConstants.RAMP_RATE);
            closedLoop.outputRange(-1, OUTPUT_MAXIMUM, ClosedLoopSlot.kSlot0) // kslot 0 is up
                .p(ElevatorConstants.P_UP, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.I_UP, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.D_UP, ClosedLoopSlot.kSlot0);
            closedLoop.maxMotion.maxVelocity(MAX_VELOCITY_UP, ClosedLoopSlot.kSlot0)
                .maxAcceleration(MAX_ACCELERATION_UP, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(ALLOWED_ERROR_UP, ClosedLoopSlot.kSlot0);

            closedLoop.outputRange(OUTPUT_MINIMUM, 1, ClosedLoopSlot.kSlot1) // kslot 1 is down
                .p(ElevatorConstants.P_DOWN, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.I_DOWN, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.D_DOWN, ClosedLoopSlot.kSlot1);
            closedLoop.maxMotion.maxVelocity(MAX_VELOCITY_DOWN, ClosedLoopSlot.kSlot1)
                .maxAcceleration(MAX_ACCELERATION_DOWN, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(ALLOWED_ERROR_DOWN, ClosedLoopSlot.kSlot1);
        }};
    }

    public static class VisionConstants {
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
        public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
        public static final Matrix<N3, N1> visionStdDevsTrust = VecBuilder.fill(0.2, 0.2, 0.2);

  }



    public static final class WristConstants{
        public static final int MOTOR_ID = 16;
        public static double STOW_ROT_WRIST = -0.1;
        public static double INTAKE_ROT_WRIST = -3.957139;
        public static double L1_ROT_WRIST = -13.47621822;
        public static double L2_ROT_WRIST = -13.738119125366211;
        public static double L3_ROT_WRIST = -16.38093376;
        public static double L4_ROT_WRIST = -11.690532684326172;

        public static final double P = 0.03;
        public static final double I = 0.0;
        public static final double D = 0.01;

        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 30;
        public static final double RAMP_RATE = 0.5;
        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig() {{
            idleMode(IdleMode.kBrake);
            smartCurrentLimit(WristConstants.CURRENT_LIMIT);
            inverted(WristConstants.MOTOR_INVERTED);
            openLoopRampRate(WristConstants.RAMP_RATE);
            closedLoop.p(WristConstants.P);
            closedLoop.i(WristConstants.I);
            closedLoop.d(WristConstants.D);
        }};

    }
    public static final class EndAffectorConstants{
        public static final int MOTOR_ID = 15;

        public static final double INTAKE_CORAL_SPEED = -0.4;
        public static final double HOLD_CORAL_SPEED = -0.1;
        public static final double DEPOSIT_CORAL_SPEED = 0.45;

        public static final boolean INTAKE_INVERTED = false;
        public static final int INTAKE_CURRENT_LIMIT = 30;
        public static final double INTAKE_RAMP_RATE = 0.5;
        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig() {{
            idleMode(IdleMode.kBrake);
            smartCurrentLimit(EndAffectorConstants.INTAKE_CURRENT_LIMIT);
            inverted(EndAffectorConstants.INTAKE_INVERTED);
            openLoopRampRate(EndAffectorConstants.INTAKE_RAMP_RATE);
            limitSwitch.reverseLimitSwitchEnabled(true);
        }};




    }
    
}
    
