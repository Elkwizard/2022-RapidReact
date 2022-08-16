package com.spartronics4915.lib.commands;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.spartronics4915.lib.subsystems.drive.AbstractDrive;
import com.spartronics4915.lib.subsystems.estimator.PoseEstimator;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TrajectoryFollowerCommands {
	/**
	 * Commands for generating & follow trajectories
	 * 
	 * Note: All poses and distances are in meters
	 */

	private final double mTrackWidth; // meters
	private final PoseEstimator mPoseEstimator;
	private final AbstractDrive mDrive;
	private final BiConsumer<Double, Double> setMotorSpeeds;

	public TrajectoryFollowerCommands(
		AbstractDrive drive,
		PoseEstimator poseEstimator,
		double trackWidth // meters
	) {
		mDrive = drive;
		mPoseEstimator = poseEstimator;
		mTrackWidth = trackWidth;

		setMotorSpeeds = (Double a, Double b) -> {
			Logger.info("Wheel speeds: " + a + "m/s, " + b + "m/s");
			Logger.info("At: " + poseEstimator.getPose());
			mDrive.getLeftMotor().setVelocity(a);
			mDrive.getRightMotor().setVelocity(b);
		};
	}

	public class FollowTrajectory extends RamseteCommand {
		public FollowTrajectory(
			ArrayList<Pose2d> waypoints, // meters
			double startVelocity, double endVelocity, // meters per second
			double maxVelocity, double maxAccel // meters per second
		) {
			super(
				TrajectoryGenerator.generateTrajectory(
					waypoints,
					new TrajectoryConfig(maxVelocity, maxAccel)
						.setStartVelocity(startVelocity)
						.setEndVelocity(endVelocity)
				),
				mPoseEstimator::getPose,
				new RamseteController(),
				new DifferentialDriveKinematics(mTrackWidth),
				setMotorSpeeds,
				mDrive
			);

		}
	}
}
