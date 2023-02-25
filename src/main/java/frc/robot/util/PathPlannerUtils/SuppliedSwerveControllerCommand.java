package frc.robot.util.PathPlannerUtils;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/** Custom PathPlanner version of SwerveControllerCommand */
public class SuppliedSwerveControllerCommand extends CommandBase {
	private final Timer timer = new Timer();
	private final Supplier<PathPlannerTrajectory> trajectorySupplier;
	private final Supplier<Pose2d> poseSupplier;
	private final SwerveDriveKinematics kinematics;
	private final PPHolonomicDriveController controller;
	private final Consumer<SwerveModuleState[]> outputModuleStates;
	private final Consumer<ChassisSpeeds> outputChassisSpeeds;
	private final boolean useKinematics;
	private final boolean useAllianceColor;
	private final Field2d field = new Field2d();

	private SwerveSubsystem swerve;

	private PathPlannerTrajectory transformedTrajectory;

	/**
	 * Constructs a new PPSwerveControllerCommand that when executed will follow the
	 * provided
	 * trajectory. This command will not return output voltages but ChassisSpeeds
	 * from the position
	 * controllers which need to be converted to module states and put into a
	 * velocity PID.
	 *
	 * <p>
	 * Note: The controllers will *not* set the output to zero upon completion of
	 * the path this is
	 * left to the user, since it is not appropriate for paths with nonstationary
	 * endstates.
	 *
	 * @param trajectory          The trajectory to follow.
	 * @param poseSupplier        A function that supplies the robot pose - use one
	 *                            of the odometry classes
	 *                            to provide this.
	 * @param xController         The Trajectory Tracker PID controller for the
	 *                            robot's x position.
	 * @param yController         The Trajectory Tracker PID controller for the
	 *                            robot's y position.
	 * @param rotationController  The Trajectory Tracker PID controller for angle
	 *                            for the robot.
	 * @param outputChassisSpeeds The field relative chassis speeds output consumer.
	 * @param useAllianceColor    Should the path states be automatically
	 *                            transformed based on alliance
	 *                            color? In order for this to work properly, you
	 *                            MUST create your path on the blue side of
	 *                            the field.
	 * @param requirements        The subsystems to require.
	 */
	public SuppliedSwerveControllerCommand(
			Supplier<PathPlannerTrajectory> trajectory,
			Supplier<Pose2d> poseSupplier,
			PIDController xController,
			PIDController yController,
			PIDController rotationController,
			Consumer<ChassisSpeeds> outputChassisSpeeds,
			boolean useAllianceColor,
			Subsystem... requirements) {
		this.trajectorySupplier = trajectory;
		this.poseSupplier = poseSupplier;
		this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
		this.outputChassisSpeeds = outputChassisSpeeds;
		this.outputModuleStates = null;
		this.kinematics = null;
		this.useKinematics = false;
		this.useAllianceColor = useAllianceColor;

		addRequirements(requirements);

	}

	/**
	 * Constructs a new PPSwerveControllerCommand that when executed will follow the
	 * provided
	 * trajectory. This command will not return output voltages but ChassisSpeeds
	 * from the position
	 * controllers which need to be converted to module states and put into a
	 * velocity PID.
	 *
	 * <p>
	 * Note: The controllers will *not* set the output to zero upon completion of
	 * the path this is
	 * left to the user, since it is not appropriate for paths with nonstationary
	 * endstates.
	 *
	 * @param trajectory          The trajectory to follow.
	 * @param poseSupplier        A function that supplies the robot pose - use one
	 *                            of the odometry classes
	 *                            to provide this.
	 * @param xController         The Trajectory Tracker PID controller for the
	 *                            robot's x position.
	 * @param yController         The Trajectory Tracker PID controller for the
	 *                            robot's y position.
	 * @param rotationController  The Trajectory Tracker PID controller for angle
	 *                            for the robot.
	 * @param outputChassisSpeeds The field relative chassis speeds output consumer.
	 * @param requirements        The subsystems to require.
	 */
	public SuppliedSwerveControllerCommand(
			Supplier<PathPlannerTrajectory> trajectory,
			Supplier<Pose2d> poseSupplier,
			PIDController xController,
			PIDController yController,
			PIDController rotationController,
			Consumer<ChassisSpeeds> outputChassisSpeeds,
			Subsystem... requirements) {
		this(
				trajectory,
				poseSupplier,
				xController,
				yController,
				rotationController,
				outputChassisSpeeds,
				true,
				requirements);
	}

	/**
	 * Constructs a new PPSwerveControllerCommand that when executed will follow the
	 * provided
	 * trajectory. This command will not return output voltages but rather raw
	 * module states from the
	 * position controllers which need to be put into a velocity PID.
	 *
	 * <p>
	 * Note: The controllers will *not* set the output to zero upon completion of
	 * the path- this is
	 * left to the user, since it is not appropriate for paths with nonstationary
	 * endstates.
	 *
	 * @param trajectory         The trajectory to follow.
	 * @param poseSupplier       A function that supplies the robot pose - use one
	 *                           of the odometry classes
	 *                           to provide this.
	 * @param kinematics         The kinematics for the robot drivetrain.
	 * @param xController        The Trajectory Tracker PID controller for the
	 *                           robot's x position.
	 * @param yController        The Trajectory Tracker PID controller for the
	 *                           robot's y position.
	 * @param rotationController The Trajectory Tracker PID controller for angle for
	 *                           the robot.
	 * @param outputModuleStates The raw output module states from the position
	 *                           controllers.
	 * @param useAllianceColor   Should the path states be automatically transformed
	 *                           based on alliance
	 *                           color? In order for this to work properly, you MUST
	 *                           create your path on the blue side of
	 *                           the field.
	 * @param requirements       The subsystems to require.
	 */
	public SuppliedSwerveControllerCommand(
			Supplier<PathPlannerTrajectory> trajectory,
			Supplier<Pose2d> poseSupplier,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			PIDController rotationController,
			Consumer<SwerveModuleState[]> outputModuleStates,
			boolean useAllianceColor,
			SwerveSubsystem s,
			Subsystem... requirements) {
		this.trajectorySupplier = trajectory;
		this.poseSupplier = poseSupplier;
		this.kinematics = kinematics;
		this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
		this.outputModuleStates = outputModuleStates;
		this.outputChassisSpeeds = null;
		this.useKinematics = true;
		this.useAllianceColor = useAllianceColor;
		this.swerve = s;
		addRequirements(requirements);

	}

	/**
	 * Constructs a new PPSwerveControllerCommand that when executed will follow the
	 * provided
	 * trajectory. This command will not return output voltages but rather raw
	 * module states from the
	 * position controllers which need to be put into a velocity PID.
	 *
	 * <p>
	 * Note: The controllers will *not* set the output to zero upon completion of
	 * the path- this is
	 * left to the user, since it is not appropriate for paths with nonstationary
	 * endstates.
	 *
	 * @param trajectory         The trajectory to follow.
	 * @param poseSupplier       A function that supplies the robot pose - use one
	 *                           of the odometry classes
	 *                           to provide this.
	 * @param kinematics         The kinematics for the robot drivetrain.
	 * @param xController        The Trajectory Tracker PID controller for the
	 *                           robot's x position.
	 * @param yController        The Trajectory Tracker PID controller for the
	 *                           robot's y position.
	 * @param rotationController The Trajectory Tracker PID controller for angle for
	 *                           the robot.
	 * @param outputModuleStates The raw output module states from the position
	 *                           controllers.
	 * @param requirements       The subsystems to require.
	 */
	public SuppliedSwerveControllerCommand(
			Supplier<PathPlannerTrajectory> trajectory,
			Supplier<Pose2d> poseSupplier,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			PIDController rotationController,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem s,
			Subsystem... requirements) {
		this(
				trajectory,
				poseSupplier,
				kinematics,
				xController,
				yController,
				rotationController,
				outputModuleStates,
				true,
				s,
				requirements);
	}

	@Override
	public void initialize() {

		controller.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d(0.25)));

		PathPlannerTrajectory trajectory = trajectorySupplier.get();

		swerve.resetOdometry(trajectory.getInitialHolonomicPose());

		if (useAllianceColor && trajectory.fromGUI) {
			transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
					trajectory, DriverStation.getAlliance());
		} else {
			transformedTrajectory = trajectory;
		}

		this.field.getObject("traj").setTrajectory(transformedTrajectory);

		this.timer.reset();
		this.timer.start();

		PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
	}

	@Override
	public void execute() {
		double currentTime = this.timer.get();
		PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

		Pose2d currentPose = this.poseSupplier.get();
		this.field.setRobotPose(currentPose);
		PathPlannerServer.sendPathFollowingData(
				new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
				currentPose);

		// SmartDashboard.putNumber(
		// 		"PPSwerveControllerCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
		// SmartDashboard.putNumber(
		// 		"PPSwerveControllerCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
		// SmartDashboard.putNumber(
		// 		"PPSwerveControllerCommand_rotationError",
		// 		currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

		ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

		if (this.useKinematics) {
			SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

			this.outputModuleStates.accept(targetModuleStates);
		} else {
			this.outputChassisSpeeds.accept(targetChassisSpeeds);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.timer.stop();

		if (interrupted) {
			if (useKinematics) {
				this.outputModuleStates.accept(
						this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
			} else {
				this.outputChassisSpeeds.accept(new ChassisSpeeds());
			}
		}
	}

	@Override
	public boolean isFinished() {
		return controller.atReference() && this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
	}
}
