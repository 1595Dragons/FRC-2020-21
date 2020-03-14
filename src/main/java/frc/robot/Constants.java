package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TODO Documentation
 *
 * @author Spud
 */
public class Constants {

	/**
	 * Setpoint Shooting constants
	 */
	public static final double shooterPower = 0.7d;
	public static final Constants feedSpeed = new Constants("feedSpeed", 5700d),
			feedRatio = new Constants("feedRatio", 0.2d);

	/**
	 * Limelight and navX constants
	 */
	public static final Constants taShoot = new Constants("taShoot", 0.1d),
			txShoot = new Constants("txShoot", 0.0d),
			tyShoot = new Constants("tyShoot", 0.0d),
			tx_kp = new Constants("tx_kp", 0.017d),
			tx_ki = new Constants("tx_ki", 0.00002d),
			tx_kd = new Constants("tx_kd", 0.0d),
			tx_iMax = new Constants("tx_iMax", 0.02d),
			ty_kp = new Constants("ty_kp", 0.016d),
			ty_ki = new Constants("ty_ki", 0.0001d),
			ty_kd = new Constants("ty_kd", 0.0d),
			ahrs_kp = new Constants("ahrs_kp", 0.0002d),
			ahrs_ki = new Constants("ahrs_ki", 0.000001d),
			ahrs_kd = new Constants("ahrs_kd", 0.000001d);

	/**
	 * Shooter constants
	 */
	public static final double visionCenterHeight = 89.4d, //should be 89.75 base on official guild
			cameraHeight = 6.6d, cameraAngle = 31d, distanceModifier = 1.115d,
			gearRatioLow = 14.88d, gearRatioHigh = 6.55d, wheelRotationToInch = Math.PI * 6, shooterHeight = 20.0d,
			g = 386.2d, goalHeight = 98.25d, shooterAngleMax = 65.0d, shooterAngleMin = 30.0d, goalDepth = 29.25d;
	public static final Constants targetDistance = new Constants("targetDistance", 120.0d);

	/**
	 * PID Constants (What are these used for?)
	 */
	public static final double maxRPM = 5700d, dp_kp = 0.0018d,
			dp_ki = 0.000002d, dp_kd = 0.000001d,
			dp_kiz = 500d, dp_kff = 0.000175d;

	/**
	 * Shooter PID constants
	 */
	public static final Constants s_kp = new Constants("s_kp", 0.00010d),
			s_ki = new Constants("s_ki", 0.0000001d),
			s_kd = new Constants("s_kd", 0.0d),
			s_kiz = new Constants("s_kiz", 1000d),
			s_kff = new Constants("s_kff", 0.000175d),
			s_kMaxOutput = new Constants("s_kMaxOutput", 0.0d),
			s_kMinOutput = new Constants("s_kMinOutput", -1.0d),
			s_iAccum = new Constants("s_iAccum", 0.0d);

	/**
	 * Revolver PID constants
	 */
	public static final Constants r_kp = new Constants("r_kp", 0.0001d),
			r_ki = new Constants("r_ki", 0.000000d),
			r_kd = new Constants("r_kd", 0.0d),
			r_kiz = new Constants("r_kiz", 0.0d),
			r_kff = new Constants("r_kff", 0.0002d);

	/**
	 * Feeder PID constants
	 */
	public static final Constants f_kp = new Constants("f_kp", 0.0001d),
			f_ki = new Constants("f_ki", 0.0000010d),
			f_kd = new Constants("f_kd", 0.0d),
			f_kiz = new Constants("f_kiz", 1500.0d),
			f_kff = new Constants("f_kff", 0.00019d);

	/**
	 * Drivertrain PID Constants
	 */
	public static final Constants dv_kp = new Constants("dv_kp", 0.000005d),
			dv_ki = new Constants("dv_ki", 0.0000005d),
			dv_kd = new Constants("dv_kd", 0.000001d),
			dv_kiz = new Constants("dv_kiz", 500.0d),
			dv_kff = new Constants("dv_kff", 0.000175d);

	/**
	 * TODO Documentation
	 */
	protected String key;

	/**
	 * TODO Documentation
	 */
	private double value;

	/**
	 * TODO Documentation
	 *
	 * @param key
	 * @param defaultValue
	 */
	public Constants(String key, double defaultValue) {
		this.key = key;
		this.value = defaultValue;
		SmartDashboard.putNumber(this.key, this.value);
	}

	/**
	 * TODO Documentation
	 *
	 * @return
	 */
	public double getValue() {
		return SmartDashboard.getNumber(this.key, this.value);
	}

	/**
	 * TODO Documentation
	 *
	 * @param constant
	 * @return
	 */
	public static double getValue(Constants constant) {
		return SmartDashboard.getNumber(constant.key, constant.value);
	}
}
