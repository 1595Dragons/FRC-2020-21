package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

/**
 * TODO Documentation
 */
public class RobotMap {

	/**
	 * TODO Documentation
	 */
	public MiniPID tx_Pid = new MiniPID(Constants.tx_kp.getValue(), Constants.tx_ki.getValue(), Constants.tx_kd.getValue()),
			ty_Pid = new MiniPID(Constants.ty_kp.getValue(), Constants.ty_ki.getValue(), Constants.ty_kd.getValue()),
			ahrs_Pid = new MiniPID(Constants.ahrs_kp.getValue(), Constants.ahrs_ki.getValue(), Constants.ahrs_kd.getValue());

	/**
	 * TODO Documentation
	 */
	public CANSparkMax leftDrive, rightDrive, leftDrive2, rightDrive2, shooter1, shooter2, feeder, revolver, intaker;

	/**
	 * TODO Documentation
	 */
	public CANPIDController s_pidController, f_pidController, r_pidController, leftdrive_pidController;

	/**
	 * TODO Documentation
	 */
	public CANEncoder s_encoder, f_encoder, r_encoder, leftdrive_pidController_encoder;

	/**
	 * TODO Documentation
	 */
	public Servo shooterServo1, shooterServo2;

	/**
	 * TODO Documentation
	 */
	public DoubleSolenoid shifter, pto, intakeExtension;

	/**
	 * TODO Documentation
	 */
	public CANPIDController rightdrive_pidController;

	/**
	 * TODO Documentation
	 */
	public CANEncoder rightdrive_pidController_encoder;

	/**
	 * TODO Documentation
	 */
	private static final int shooter1Port = 5, shooter2Port = 8, feederPort = 7, revolverPort = 9, intakerPort = 6,
			leftDrive1Port = 1, leftDrive2Port = 2, rightDrive1Port = 3, rightDrive2Port = 4, shooterServo1Port = 1,
			shooterServo2Port = 2, shifterPort1 = 2, shifterPort2 = 6, ptoPort1 = 1, ptoPort2 = 5,
			intakeExtentionPort1 = 3, intakeExtentionPort2 = 7;

	/**
	 * TODO Documentation
	 */
	public NetworkTable limelight;

	/**
	 * TODO Documentation
	 */
	public double limelight2MaxY = 24.85, limelight2MaxX = 29.8;

	/**
	 * TODO Documentation
	 */
	public final XboxController driver = new XboxController(0), operator = new XboxController(1);

	RobotMap() {

		this.declareMotors();

		this.setupMotorBehavior();

		this.rightdrive_pidController = this.rightDrive.getPIDController();
		this.leftdrive_pidController = this.leftDrive.getPIDController();

		this.rightdrive_pidController_encoder = this.rightDrive.getEncoder();
		this.leftdrive_pidController_encoder = this.leftDrive.getEncoder();

		this.resetDrivePIDs();

		this.f_pidController = this.feeder.getPIDController();
		this.s_pidController = this.shooter1.getPIDController();
		this.r_pidController = this.revolver.getPIDController();

		this.f_encoder = this.feeder.getEncoder();
		this.s_encoder = this.shooter1.getEncoder();
		this.r_encoder = this.revolver.getEncoder();

		this.s_pidController.setIMaxAccum(Constants.s_iAccum.getValue(), 0);

		this.setupSRF();

		this.limelight = NetworkTableInstance.getDefault().getTable("limelight");

		this.shooterServo1 = new Servo(RobotMap.shooterServo1Port);
		this.shooterServo2 = new Servo(RobotMap.shooterServo2Port);

		this.shifter = new DoubleSolenoid(RobotMap.shifterPort1, RobotMap.shifterPort2);
		this.pto = new DoubleSolenoid(RobotMap.ptoPort1, RobotMap.ptoPort2);
		this.intakeExtension = new DoubleSolenoid(RobotMap.intakeExtentionPort1, RobotMap.intakeExtentionPort2);
	}

	/**
	 * Resets the drivetrain PIDs
	 */
	public void resetDrivePIDs() {
		this.resetDrivePIDs(1.0d);
	}

	/**
	 * TODO Documentation
	 *
	 * @param max TODO
	 */
	public void resetDrivePIDs(double max) {
		// Reset the right drivetrain
		this.setUpPIDController(this.rightdrive_pidController, Constants.dv_kp.getValue(), Constants.dv_ki.getValue(),
				Constants.dv_kd.getValue(), Constants.dv_kiz.getValue(), Constants.dv_kff.getValue(), max, -max);

		// Reset the left drivetrain
		this.setUpPIDController(this.leftdrive_pidController, Constants.dv_kp.getValue(), Constants.dv_ki.getValue(),
				Constants.dv_kd.getValue(), Constants.dv_kiz.getValue(), Constants.dv_kff.getValue(), max, -max);
	}

	/**
	 * Setup the PIDs for the shooter, revolver, and feeder.
	 */
	public void setupSRF() {
		// Setup the shooter PID
		this.setUpPIDController(this.s_pidController, Constants.s_kp.getValue(), Constants.s_ki.getValue(),
				Constants.s_kd.getValue(), Constants.s_kiz.getValue(), Constants.s_kff.getValue(),
				Constants.s_kMaxOutput.getValue(), Constants.s_kMinOutput.getValue());

		// Do the same for the revolver
		this.setUpPIDController(this.r_pidController, Constants.r_kp.getValue(), Constants.r_ki.getValue(),
				Constants.r_kd.getValue(), Constants.r_kiz.getValue(), Constants.r_kff.getValue(), 1, -1);

		// Finish it off with the feeder
		this.setUpPIDController(this.f_pidController, Constants.f_kp.getValue(), Constants.f_ki.getValue(),
				Constants.f_kd.getValue(), Constants.f_kiz.getValue(), Constants.f_kff.getValue(), 1, -1);
	}

	/**
	 * Declare the motors used on the robot.
	 */
	private void declareMotors() {
		this.leftDrive = new CANSparkMax(RobotMap.leftDrive1Port, MotorType.kBrushless);
		this.leftDrive2 = new CANSparkMax(RobotMap.leftDrive2Port, MotorType.kBrushless);
		this.rightDrive = new CANSparkMax(RobotMap.rightDrive1Port, MotorType.kBrushless);
		this.rightDrive2 = new CANSparkMax(RobotMap.rightDrive2Port, MotorType.kBrushless);
		this.shooter1 = new CANSparkMax(RobotMap.shooter1Port, MotorType.kBrushless);
		this.shooter2 = new CANSparkMax(RobotMap.shooter2Port, MotorType.kBrushless);
		this.feeder = new CANSparkMax(RobotMap.feederPort, MotorType.kBrushless);
		this.revolver = new CANSparkMax(RobotMap.revolverPort, MotorType.kBrushless);
	}

	/**
	 * Sets up specific motor behavior.
	 * <p>
	 * Note: Motors must be declared first!
	 */
	private void setupMotorBehavior() {
		this.leftDrive2.follow(this.leftDrive);
		this.rightDrive2.follow(this.rightDrive);
		this.shooter2.follow(this.shooter1, true);
		this.rightDrive.setInverted(true);
		this.leftDrive.setSmartCurrentLimit(40);
		this.rightDrive.setSmartCurrentLimit(40);
		this.shooter1.setSmartCurrentLimit(40);
		this.revolver.setSmartCurrentLimit(40);
		this.feeder.setSmartCurrentLimit(40);
	}

	/**
	 * TODO Documentation and comments
	 */
	public void setupPIDs() {
		this.setupSRF();

		this.leftdrive_pidController_encoder.setPosition(0);
		this.rightdrive_pidController_encoder.setPosition(0);

		this.resetDrivePIDs(0.8d);

		this.tx_Pid.reset();
		this.tx_Pid.setPID(Constants.tx_kp.getValue(), Constants.tx_ki.getValue(), Constants.tx_kd.getValue());
		this.tx_Pid.setMaxIOutput(Constants.tx_iMax.getValue());
		this.tx_Pid.setSetpoint(Constants.txShoot.getValue());
		this.tx_Pid.setOutputLimits(0.4);

		this.ty_Pid.reset();
		this.ty_Pid.setPID(Constants.ty_kp.getValue(), Constants.ty_ki.getValue(), Constants.ty_kd.getValue());
		this.ty_Pid.setSetpoint(Constants.targetDistance.getValue());
		this.ty_Pid.setOutputLimits(0.4);

		this.ahrs_Pid.reset();
		this.ahrs_Pid.setPID(Constants.ahrs_kp.getValue(), Constants.ahrs_ki.getValue(), Constants.ahrs_kd.getValue());
		this.ahrs_Pid.setOutputLimits(0.5);
	}

	/**
	 * TODO Documentation
	 *
	 * @param controller TODO
	 * @param kp         TODO
	 * @param ki         TODO
	 * @param kd         TODO
	 * @param kiz        TODO
	 * @param kff        TODO
	 * @param max        TODO
	 * @param min        TODO
	 */
	public void setUpPIDController(CANPIDController controller, double kp, double ki, double kd, double kiz, double kff, double max, double min) {
		controller.setP(kp);
		controller.setI(ki);
		controller.setD(kd);
		controller.setIZone(kiz);
		controller.setFF(kff);
		controller.setOutputRange(min, max);
	}
}
	
