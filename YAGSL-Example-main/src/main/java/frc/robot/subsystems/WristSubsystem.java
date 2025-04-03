package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

     private final SparkBase angleMotor;
     private RelativeEncoder angleEncoder;
     private SparkClosedLoopController angleClosedLoop;
     private double holdAngle;
     public enum WristPosition {stow, L1, L2, L3, L4, IN};
     public WristSubsystem ()
     {
       
          angleMotor = new SparkMax(WristConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless); 
          angleMotor.configure(WristConstants.MOTOR_CONFIG,
               SparkBase.ResetMode.kResetSafeParameters,
               SparkBase.PersistMode.kPersistParameters
          );   
          angleClosedLoop = angleMotor.getClosedLoopController();
          angleEncoder = angleMotor.getEncoder();
          holdAngle = WristConstants.STOW_ROT_WRIST;
     }

     @Override
     public void periodic() {
         SmartDashboard.putNumber("Hold Angle", holdAngle);
         SmartDashboard.putNumber("Wrist Encoder Angle", getAngle());
     }

     /**
      * This gives us the angle of the motor
      * @return The current angle of the wrist in degrees
      */
     public double getAngle()
     {
          return angleEncoder.getPosition();
     }

     /**
     * This gives us the hold angle of the PID controller
     * @return The current hold angle of the PID controller in degrees
     */
     public double getHoldAngle()
     {
         return holdAngle;
     }

     public void setAngle(double angle)
     {
          holdAngle = angle;
          angleClosedLoop.setReference(holdAngle, ControlType.kPosition);
     }  

     public void setHoldAngle()
     {
          angleClosedLoop.setReference(holdAngle, ControlType.kPosition);
     }

     public Command AquirePositionCommand()
     {
         return new InstantCommand(()->setAngle(WristConstants.INTAKE_ROT_WRIST));
     }     
     public Command StowPositionCommand(){
          return new InstantCommand(()->setAngle(WristConstants.STOW_ROT_WRIST)); 
     }
     public Command HoldPositionCommand()
     {
          return new RunCommand(()->setHoldAngle(),this);
     }

    public Command goTo(WristPosition position) {
        switch (position) {
            case stow:
                System.out.println("goTo - stow");
                return new InstantCommand(()->setAngle(WristConstants.STOW_ROT_WRIST));
            case L1:
            System.out.println("goTo - L1");
                return new InstantCommand(()->setAngle(WristConstants.L1_ROT_WRIST));
            case L2:
            System.out.println("goTo - L2");
                return new InstantCommand(()->setAngle(WristConstants.L2_ROT_WRIST));
            case L3:
            System.out.println("goTo - L3");
                return new InstantCommand(()->setAngle(WristConstants.L3_ROT_WRIST));
            case L4:
            System.out.println("goTo - L4");
                return new InstantCommand(()->setAngle(WristConstants.L4_ROT_WRIST));
            case IN:
            System.out.println("goTo - IN");
                return new InstantCommand(()->setAngle(WristConstants.INTAKE_ROT_WRIST));
            default:
                System.out.println("default");  
                return new InstantCommand(()->setAngle(WristConstants.STOW_ROT_WRIST));
        }
    }



}
