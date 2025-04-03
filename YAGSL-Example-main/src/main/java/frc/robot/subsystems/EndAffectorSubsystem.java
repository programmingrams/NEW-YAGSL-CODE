package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndAffectorConstants;

public class EndAffectorSubsystem extends SubsystemBase {
        /** false when there's a coral in the early stage of the mechanism mechanism */
        SparkLimitSwitch intakeCoralBeamBreak;
        /** false when there's a coral in the later stage of the mechanism */
        DigitalInput placementCoralBeamBreak;
        /** runs backwards to pull algae in, and forward to pull the game piece out or coral in */
        SparkBase intakeMotor;
    
        public EndAffectorSubsystem() {
            intakeMotor = new SparkMax(EndAffectorConstants.MOTOR_ID, MotorType.kBrushless);
            intakeMotor.configure(
                EndAffectorConstants.MOTOR_CONFIG, 
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            );
            intakeCoralBeamBreak = intakeMotor.getReverseLimitSwitch();
            placementCoralBeamBreak = new DigitalInput(EndAffectorConstants.PLACEMENT_CORAL_BEAM_BREAK_ID);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeHasCoral", intakeHasCoral());
        SmartDashboard.putBoolean("placementHasCoral", placementHasCoral());
    }

    /**
     * runs intake motor backwards at the specified speed to intake the game coral from the dispensary.
     */
    public void setCoralSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * runs intake motor backwards
     */


    public void depositCoral() {
        intakeMotor.set(EndAffectorConstants.DEPOSIT_CORAL_SPEED);
    }
        

    public void stopMotor() {
        intakeMotor.set(0);
    }

  
    public void holdCoral() {
        intakeMotor.set(EndAffectorConstants.HOLD_CORAL_SPEED);
    }

    /**
     * Returns the speed that the intake motor should run to continue to intake coral
     * @return the speed the motor should run
     */
    public double coralSpeed() {
        if(intakeHasCoral()) {
            if(placementHasCoral()) {
                return EndAffectorConstants.ALIGNING_CORAL_SPEED;
            }
            return EndAffectorConstants.CONVEYING_CORAL_SPEED;
        }
        if(placementHasCoral()) {
            return EndAffectorConstants.HOLD_CORAL_SPEED;
        }
        return EndAffectorConstants.INTAKE_CORAL_SPEED;
    }

    /**
     * Runs the intake until the robot has coral, slowing down as coral progresses through the system
     * @return the command
     */
    public Command intakeCoralCommand() {
        return new RunCommand(() -> setCoralSpeed(coralSpeed()))
            .until(this::hasCoral)
            .andThen(new InstantCommand(this::stopMotor));
    }

}
