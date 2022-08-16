package com.spartronics4915.lib.subsystems.estimator;

import com.spartronics4915.lib.subsystems.drive.AbstractDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
	/**
	 * Estimates the position of the robot based exclusively on an IMU without vision.
	 * All units are in meters.
	 * 
	 * Note: all poses refer to the center of the robot
	 */
	
	private DifferentialDriveOdometry mOdometry;
	private AbstractDrive mDrivetrain;
	
	public PoseEstimator(AbstractDrive _drivetrain, Pose2d initialPose) {
		mDrivetrain = _drivetrain;
		mOdometry = new DifferentialDriveOdometry(
			mDrivetrain.getIMUHeading(), initialPose
		);
	}

	public PoseEstimator(AbstractDrive _drivetrain) {
		this(_drivetrain, new Pose2d());
	}

	@Override
	public void periodic() {
		getPose();
	}

	public Pose2d getPose() {
		return mOdometry.update(
			mDrivetrain.getIMUHeading(),
			mDrivetrain.getLeftMotor().getEncoder().getPosition(),
			mDrivetrain.getRightMotor().getEncoder().getPosition()
		);
	}
}
