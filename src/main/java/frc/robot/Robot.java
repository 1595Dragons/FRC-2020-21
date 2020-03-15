package frc.robot;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	private RobotMap robot = new RobotMap();

	private boolean shooterReady = false, usePreSetShooterPower = true;

	private double shooterAngle, shooterVelocity, shooterSetPoint;

	// Auto stuff
	public Timer autoTimer = new Timer();
	private double autoEndTime = 3d, autoTargetPosition1 = 20d, servoPosition1 = 0d, servoPosition2 = 0d;
	private boolean autoShoot = true;

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {

		this.autoTimer.start();

		this.shooterReady = false;

		this.robot.setupPIDs();

		this.robot.limelight.getEntry("pipeline").setNumber(0);
		this.autoTimer.reset();
		this.autoTimer.start();
		this.autoEndTime = SmartDashboard.getNumber("autoEndTime", this.autoEndTime);
		this.autoShoot = SmartDashboard.getBoolean("autoShoot", this.autoShoot);

		this.shooterReady = false;

		this.autoTargetPosition1 = SmartDashboard.getNumber("autoTargetPosition1", this.autoTargetPosition1);
		this.usePreSetShooterPower = SmartDashboard.getBoolean("usePreSetShooterPower", this.usePreSetShooterPower);

		this.autoTimer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		while (this.autoTimer.get() < 4) {
			SmartDashboard.putNumber("Timer", this.autoTimer.get());

			if (this.shooterReady) {
				this.robot.f_pidController.setReference(-Constants.feedSpeed.getValue(), ControlType.kVelocity);
			} else {
				this.robot.f_pidController.setReference(0, ControlType.kVelocity);
			}

			this.robot.r_pidController.setReference(Constants.feedSpeed.getValue() * Constants.feedRatio.getValue(),
					ControlType.kVelocity);
			this.robot.s_pidController.setReference(-Constants.maxRPM * Constants.shooterPower, ControlType.kVelocity);

			this.shooterReady = Math.abs(-this.robot.s_encoder.getVelocity() - Constants.maxRPM * Constants.shooterPower) < 250;
		}


		this.robot.f_pidController.setReference(0, ControlType.kVelocity);
		this.robot.r_pidController.setReference(0, ControlType.kVelocity);
		this.robot.s_pidController.setReference(0, ControlType.kVelocity);
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {

		this.robot.setupPIDs();

		this.usePreSetShooterPower = SmartDashboard.getBoolean("usePreSetShooterPower", this.usePreSetShooterPower);
		this.robot.s_pidController.setIMaxAccum(Constants.s_iAccum.getValue(), 0);

		this.robot.limelight.getEntry("pipeline").setNumber(0);
		this.shooterReady = false;
		this.robot.leftdrive_pidController_encoder.setPosition(0);
		this.robot.rightdrive_pidController_encoder.setPosition(0);
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {

		if (!this.robot.operator.getBButton() && !this.robot.operator.getAButton()) {
			this.robot.r_pidController.setReference(800 - 800 * this.robot.operator.getY(Hand.kLeft),
					ControlType.kVelocity);
		}

		if (this.robot.operator.getAButton() && !this.robot.operator.getBButton()) {
			this.robot.r_pidController.setReference(-800, ControlType.kVelocity);
		}

		if (this.robot.operator.getXButton() && !this.robot.operator.getBButton()) {
			this.robot.f_pidController.setReference(200, ControlType.kVelocity);
		} else if (!this.robot.operator.getBButton()) {
			this.robot.f_pidController.setReference(0, ControlType.kVelocity);
		}

		double tv = this.robot.limelight.getEntry("tv").getDouble(0),
				tx = this.robot.limelight.getEntry("tx").getDouble(0),
				ty = this.robot.limelight.getEntry("ty").getDouble(0),
				distance = (Constants.visionCenterHeight - Constants.cameraHeight) /
						Math.tan(Math.toRadians(Constants.cameraAngle + ty)) * Constants.distanceModifier;

		this.drive(distance, tx, tv);

		this.shift();

		this.checkShooter(distance);

		if (robot.operator.getBumper(Hand.kLeft)) {
			robot.intakeExtension.set(Value.kReverse);
		}

		if (robot.operator.getBumper(Hand.kRight)) {
			robot.intakeExtension.set(Value.kForward);
		}

		if (robot.driver.getXButtonPressed()) {
			this.robot.tx_Pid.reset();
			this.robot.ty_Pid.reset();
		}

		SmartDashboard.putNumber("Feeder RPM", this.robot.f_encoder.getVelocity());
		SmartDashboard.putNumber("Shooter RPM", this.robot.s_encoder.getVelocity());
		SmartDashboard.putNumber("Revolver RPM", this.robot.r_encoder.getVelocity());
		SmartDashboard.putNumber("shooterAngle", Math.toDegrees(this.shooterAngle));
		SmartDashboard.putNumber("shooterVelocity", this.shooterVelocity);
		SmartDashboard.putNumber("shooterSetPoint", this.shooterSetPoint);
		SmartDashboard.putBoolean("shooterReady", this.shooterReady);
	}

	/**
	 * Gets the forward power determined by the driver's left stick (on the Y axis).
	 *
	 * @return The value of 80% of the Y axis or 0 of the value is below 0.75.
	 */
	private double getForward() {
		return (Math.abs(this.robot.driver.getY(Hand.kLeft)) > .075) ? this.robot.driver.getY(Hand.kLeft) * .8 : 0;
	}

	/**
	 * Gets the turn power determined by the driver's left stick (on the X axis).
	 *
	 * @return The value of 60% of the X axis or 0 of the value is below 0.75.
	 */
	private double getTurn() {
		return (Math.abs(this.robot.driver.getX(Hand.kRight)) > .075) ? this.robot.driver.getX(Hand.kRight) * .6 : 0;
	}

	/**
	 * Determines the drive method (vision assisted or standard), and what values to run the drive train at.
	 *
	 * @param distance The distance from the vision target (for vision assisted driving)
	 * @param tx       The center location of the vision target on the X axis (how far off center it is)
	 * @param tv       Whether a vision target is found (either a 1.0 or a 0.0)
	 */
	private void drive(double distance, double tx, double tv) {

		// Calculate the drive power for vision targets
		double visionTurnPower = this.robot.tx_Pid.getOutput(tx), visionMovePower = this.robot.ty_Pid.getOutput(distance);

		// Calculate the drive power for standard controls
		double leftPow = (this.getForward() + this.getTurn()) * Constants.maxRPM * 0.85;
		double rightPow = (this.getForward() - this.getTurn()) * Constants.maxRPM * 0.85;

		// If vision target was found try using it for aim assist (tv == 1).
		if (robot.driver.getXButton() && tv == 1.0d) {
			//just changed the direction at 2:48
			this.robot.leftdrive_pidController.setReference(Constants.maxRPM * -visionTurnPower + leftPow,
					ControlType.kVelocity);
			this.robot.rightdrive_pidController.setReference(Constants.maxRPM * visionTurnPower + rightPow,
					ControlType.kVelocity);
		} else if (robot.driver.getYButton() && tv == 1.0d) {
			this.robot.leftdrive_pidController.setReference(Constants.maxRPM * (visionMovePower + visionTurnPower),
					ControlType.kVelocity);
			this.robot.rightdrive_pidController.setReference(Constants.maxRPM * (visionMovePower - visionTurnPower),
					ControlType.kVelocity);
		} else {
			// Since no vision target was found just drive normally
			this.robot.leftdrive_pidController.setReference(leftPow, ControlType.kVelocity);
			this.robot.rightdrive_pidController.setReference(rightPow, ControlType.kVelocity);
		}
	}

	/**
	 * Performs the shifting from low to high gear (and vice-versa) if a RPM threshold (1.5k rmp) is met.
	 */
	private void shift() {
		if (Math.abs(this.robot.leftdrive_pidController_encoder.getVelocity()) < 1500 &&
				Math.abs(this.robot.rightdrive_pidController_encoder.getVelocity()) < 1500) {
			// Low gear
			this.robot.shifter.set(Value.kReverse);
			SmartDashboard.putString("gear", "Low");
		} else if (Math.abs(this.robot.leftdrive_pidController_encoder.getVelocity()) > 2500 &&
				Math.abs(this.robot.rightdrive_pidController_encoder.getVelocity()) > 2500) {
			// High gear
			this.robot.shifter.set(Value.kForward);
			SmartDashboard.putString("gear", "High");
		}
	}

	/**
	 * Runs the shooter portion of the teleop program (mostly just aiming and firing)
	 *
	 * @param distance The distance of the vision target.
	 */
	private void checkShooter(double distance) {
		this.shooterReady = !this.robot.operator.getBButtonReleased();

		// Check if the shooter can be spun up.
		if (this.shooterReady) {
			this.robot.f_pidController.setReference(-Constants.feedSpeed.getValue(), ControlType.kVelocity);
		} else {
			this.robot.f_pidController.setReference(0, ControlType.kVelocity);
		}

		if (this.robot.operator.getBButton()) {
			this.robot.r_pidController.setReference(Constants.feedSpeed.getValue() * Constants.feedRatio.getValue(), ControlType.kVelocity);

			// Determine whether or not to use a preset power or just the predefined speed
			if (this.usePreSetShooterPower) {
				this.robot.s_pidController.setReference(-Constants.maxRPM * Constants.shooterPower, ControlType.kVelocity);
				this.shooterReady = Math.abs(-robot.s_encoder.getVelocity() - Constants.maxRPM * Constants.shooterPower) < 250;
			} else {
				this.robot.s_pidController.setReference(-this.shooterSetPoint, ControlType.kVelocity);
				this.shooterReady = Math.abs(-robot.s_encoder.getVelocity() - this.shooterSetPoint) < 250;
			}

			// Determine if the shooter speed is adequate to mark as ready
			this.shooterReady = Math.abs(-robot.s_encoder.getVelocity() - Constants.maxRPM * Constants.shooterPower) < 250;
		} else {
			this.robot.s_pidController.setReference(0, ControlType.kVelocity);
		}

		// Get the angle of the shot to determine the velocity to use.
		this.shooterAngle = Math.tanh(2 * (Constants.goalHeight - Constants.shooterHeight) / (distance + Constants.goalDepth));
		if (Math.toDegrees(this.shooterAngle) < Constants.shooterAngleMin) {
			this.shooterAngle = Constants.shooterAngleMin;
			shooterVelocity = Math.sqrt(1 / (Math.cos(Constants.shooterAngleMin) / (-1 / 2d * Constants.g) *
					((Constants.goalHeight - Constants.shooterHeight) - Math.tan(Constants.shooterAngleMin) * distance)));
		} else if (Math.toDegrees(this.shooterAngle) > Constants.shooterAngleMax) {
			this.shooterAngle = Constants.shooterAngleMax;
			this.shooterVelocity = Math.sqrt(1 / (Math.cos(Constants.shooterAngleMax) / (-1 / 2d * Constants.g) *
					((Constants.goalHeight - Constants.shooterHeight) - Math.tan(Constants.shooterAngleMax) * distance)));
		} else {
			this.shooterVelocity = Math.sqrt(Constants.g * (Constants.goalDepth + distance) * Math.cos(this.shooterAngle));
		}

		// Mark the setpoint
		this.shooterSetPoint = 60 * this.shooterVelocity / (2 * Math.PI);
	}

	@Override
	public void testInit() {
		this.teleopInit();
		SmartDashboard.putNumber("servoPosition1", this.servoPosition1);
		SmartDashboard.putNumber("servoPosition2", this.servoPosition2);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		this.servoPosition1 = SmartDashboard.getNumber("servoPosition1", this.servoPosition1);
		this.servoPosition2 = SmartDashboard.getNumber("servoPosition2", this.servoPosition2);
		this.robot.shooterServo1.setPosition(this.servoPosition1);
		this.robot.shooterServo2.setPosition(this.servoPosition2);
	}
}
