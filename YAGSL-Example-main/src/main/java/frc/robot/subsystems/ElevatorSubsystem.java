package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private RelativeEncoder encoder;
    private SparkBase leader;
    private SparkBase follower;
    private SparkClosedLoopController controller;
    private DigitalInput lowerLimit;
    public enum Position {stow, L1, L2, L3, L4, IN};
    public boolean isCoral = true;
    Trigger zeroTrigger; 
    double setpoint = 0;
    //private DigitalInput lowerLimit2;
    Alert invalidStateRequested;

    public ElevatorSubsystem() {
        leader = new SparkMax(ElevatorConstants.LEADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        leader.configure(
            ElevatorConstants.MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        controller = leader.getClosedLoopController();
        encoder = leader.getEncoder();
        follower = new SparkMax(ElevatorConstants.FOLLOWING_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        follower.configure(
            ElevatorConstants.MOTOR_CONFIG.follow(leader, ElevatorConstants.FOLLOWER_INVERTED_FROM_LEADER),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        lowerLimit = new DigitalInput(ElevatorConstants.LOWER_LIMIT_ID);
        zeroTrigger = new Trigger(lowerLimit::get);
        zeroTrigger.onTrue(zeroElevator());
        //lowerLimit2 = new DigitalInput(ElevatorConstants.LOWER_LIMIT_ID_2);
        invalidStateRequested = new Alert("Invalid state requested for elevator. (stow used instead)", AlertType.kError);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator isAtLimit", isAtLimit());
        SmartDashboard.putNumber("Elevator", getPositionRotations());
        SmartDashboard.putNumber("Elevator current", leader.getOutputCurrent());
        SmartDashboard.putNumber("Elevator follower current", follower.getOutputCurrent());
        SmartDashboard.putNumber("Elevator velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator output", leader.getAppliedOutput());
        SmartDashboard.putNumber("Setpoint", setpoint);
        if(Math.abs(setpoint - getPositionRotations()) < 0.25 && setpoint == ElevatorConstants.STOW_ROT)
        {
            leader.stopMotor();
        }
    }

    /**
     * Set the target position of the elevator
     * @param position the target, in rotations of the motor
     */
    public void setPositionRotations(double rotations) {
        ClosedLoopSlot slot;
        if (rotations < getPositionRotations()) {
            controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, .623); // Down case; use max motion and slot 1
        } else {
            controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0); // Up case; use plain position control, slot 0
        }
    }

    /**
     * Returns the position of the motor.
     * @return the value of the relative encoder of the motor, in rotations.
     */
    public double getPositionRotations() {
        return encoder.getPosition();
    }

    /**
     * Returns the current height
     * @return the height, in inches
     */

    /**
     * Calculates the number of rotations to be at the specified number of inches.
     * @return the number of rotations
     */

    public void setSpeed(double motorSpeed) {
        leader.set(motorSpeed);
    }

    public boolean isAtLimit() {
        return !lowerLimit.get()
        //|| lowerLimit2.get()
        ;
    }

    public Command goToStow() {
        return new InstantCommand(()->setPositionRotations(ElevatorConstants.STOW_ROT));
    }

    public Command goTo(Position position) {
        switch (position) {
            case stow:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.STOW_ROT));
            case L1:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.L1_ROT));
            case L2:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.L2_ROT));
            case L3:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.L3_ROT));
            case L4:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.L4_ROT));
            case IN:
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.IN_ROT));
            default:
                invalidStateRequested.set(true);
                return new InstantCommand(()->setPositionRotations(ElevatorConstants.STOW_ROT));
        }
    }
    public Command zeroElevator() {
        return new InstantCommand(() -> encoder.setPosition(0));
    }
}
