package com.spartronics4915.lib.subsystems.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

//zack was here
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TrajectoryFollower {
	/**
	 * Trajectory generation & following utility
	 * 
	 * Note: all poses refer to the center of the robot
	 */

	private AbstractDrive mDriveSubsystem;
	private Supplier<Pose2d> mGetRobotPose; // meters
	private double mTrackWidth; // meters

	public TrajectoryFollower(
		AbstractDrive driveSubsystem,
		Supplier<Pose2d> getRobotPose, // meters
		double trackWidth // meters
	) {
		mDriveSubsystem = driveSubsystem;
		mGetRobotPose = getRobotPose;
		mTrackWidth = trackWidth;
	}

	/**
	 * Generates a trajectory based on Pose2d waypoints and velocity specifications in metric units.
	 */
	Command generateTrajectory(
		ArrayList<Pose2d> waypoints, // meters
		double startVelocity, double endVelocity, // meters per second
		double maxVelocity, double maxAccel // meters per second
	) {
		
		TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel);
		config.setStartVelocity(startVelocity);
		config.setEndVelocity(endVelocity);

		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

		RamseteController controller = new RamseteController();

		DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(mTrackWidth);

		RamseteCommand followerCommand = new RamseteCommand(
			trajectory,
			mGetRobotPose,
			controller,
			kinematics,
			(Double a, Double b) -> {
				mDriveSubsystem.getLeftMotor().setVelocity(a);
				mDriveSubsystem.getLeftMotor().setVelocity(b);
			},
			mDriveSubsystem
		);

		return followerCommand;
	}
}
