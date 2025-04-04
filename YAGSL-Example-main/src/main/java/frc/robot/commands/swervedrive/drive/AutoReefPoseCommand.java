package frc.robot.commands.swervedrive.drive;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.FieldConstants;
import frc.robot.utilities.FieldConstants.ReefSide;
import swervelib.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoReefPoseCommand extends Command {
    private final SwerveSubsystem drivetrain;
    private final PIDController distanceController = new PIDController(1, 0, 0.0);
    private final PIDController strafeController = new PIDController(1, 0, 0.0);
    private final PIDController angleController = new PIDController(0.5, 0, 0.02);
    private final DoubleSupplier controllerX;
    private final DoubleSupplier controllerY;
    private final DoubleSupplier controllerT;
    private ReefSide position;

    /*public AutoReefPoseCommand(SwerveSubsystem drivetrain,
                               SwerveRequest.RobotCentric reefAlign,
                               DoubleSupplier controllerX,
                               DoubleSupplier controllerY,
                               DoubleSupplier controllerT,
                               ReefSide position) {
        this.drivetrain = drivetrain;
        this.reefAlign = reefAlign;
        this.position = null;
        distanceController.setTolerance(0.0);
        strafeController.setTolerance(0.0);
        angleController.setTolerance(0.0);
        angleController.enableContinuousInput(-180, 180);
        addRequirements(drivetrain);
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerT = controllerT;
    }*/

    public AutoReefPoseCommand(SwerveSubsystem drivetrain,
                               DoubleSupplier controllerX,
                               DoubleSupplier controllerY,
                               DoubleSupplier controllerT,
                               ReefSide position) {
        this.drivetrain = drivetrain;
        this.position = position;
        distanceController.setTolerance(0.0);
        strafeController.setTolerance(0.0);
        angleController.setTolerance(0.0);
        angleController.enableContinuousInput(-180, 180);
        addRequirements(drivetrain);
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerT = controllerT;
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        double distanceVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;
        Pose2d currentPose = drivetrain.getPose();
        if (position == null) {
            position = FieldConstants.getNearestReefSide(drivetrain.getPose());
        }
        Pose2d reefPose = FieldConstants.getNearestReefBranch(currentPose, position);
        reefPose = reefPose.rotateAround(reefPose.getTranslation(), Rotation2d.k180deg);
        Pose2d goal = FieldConstants.toRobotRelative(currentPose, reefPose);

        distanceController.setSetpoint(0.095);
        strafeController.setSetpoint(0);
        angleController.setSetpoint(1);

        strafeVal = distanceController.calculate(goal.getX());
        distanceVal = strafeController.calculate(goal.getY());
        rotationVal = angleController.calculate(goal.getRotation().getDegrees());
        strafeVal = 1 / (Math.exp(Math.abs(distanceVal) * 2));

        if (strafeController.atSetpoint())
            strafeVal = 0;
        if (distanceController.atSetpoint())
            distanceVal = 0;
        if (angleController.atSetpoint())
            rotationVal = 0;

        /* Drive */
        ChassisSpeeds chasseeee = new ChassisSpeeds(
            ((-1*controllerX.getAsDouble() - strafeVal) * 0.75), 
            ((-1*controllerY.getAsDouble() + distanceVal) * 1), 
            ((-1*controllerT.getAsDouble() - rotationVal) * 0.35)
        );

        drivetrain.drive(chasseeee);

    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        // If all 3 PIDs are at their target, we're done
        return distanceController.atSetpoint()
                && strafeController.atSetpoint()
                && angleController.atSetpoint();
    }

    // Called once after isFinished returns true
    protected void end() {
        // RobotContainer.candleSubsystem.setAnimate("Rainbow");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

}