package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotMap {

	
	private CANSparkMax shooter1,shooter2,feeder,revolver;
	CANPIDController s_pidController;
	CANPIDController f_pidController;
	CANPIDController r_pidController;
	CANEncoder s_encoder;
	CANEncoder f_encoder;
	CANEncoder r_encoder;
	
	private static final int shooter1ID = 1, shooter2ID = 7,feederID=2,revolverID=8;
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
	public double s_kp=0.00025,s_ki=0.000001,s_kd=0,s_kiz=3000,s_kff=0,s_kMaxOutput=1,s_kMinOutput=-1,s_maxRPM=5700;
	public double f_kp=0.00015,f_ki=0.000001,f_kd=0,f_kiz=2000,f_kff=0,f_kMaxOutput=1,f_kMinOutput=-1,f_maxRPM=5700;
	public double r_kp=0.00025,r_ki=0.000001,r_kd=0,r_kiz=2000,r_kff=0,r_kMaxOutput=1,r_kMinOutput=-1,r_maxRPM=5700;

	public TalonSRX leftDrive, rightDrive,leftDrive2, rightDrive2, leftDrive3, rightDrive3;

	private final int PracticeleftDrive1Port = 5, PracticeleftDrive2Port = 6, PracticeleftDrive3Port = 7,
			PracticerightDrive1Port = 10, PracticerightDrive2Port = 8, PracticerightDrive3Port = 11;

	public final XboxController driver = new XboxController(0), operator = new XboxController(1);

	RobotMap() {
		this.leftDrive = new TalonSRX(this.PracticeleftDrive1Port);
		this.leftDrive2 = new TalonSRX(this.PracticeleftDrive2Port);
		this.leftDrive3 = new TalonSRX(this.PracticeleftDrive3Port);
		this.rightDrive = new TalonSRX(this.PracticerightDrive1Port);
		this.rightDrive2 = new TalonSRX(this.PracticerightDrive2Port);
		this.rightDrive3 = new TalonSRX(this.PracticerightDrive3Port);

		this.shooter1=new CANSparkMax(this.shooter1ID, MotorType.kBrushless);
		this.shooter2=new CANSparkMax(this.shooter2ID, MotorType.kBrushless);
		this.feeder=new CANSparkMax(this.feederID, MotorType.kBrushless);
		this.revolver=new CANSparkMax(this.revolverID, MotorType.kBrushless);
		

		//this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		//this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		this.leftDrive2.set(ControlMode.Follower, this.leftDrive.getDeviceID());
		this.leftDrive3.set(ControlMode.Follower, this.leftDrive.getDeviceID());
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive.getDeviceID());
		this.rightDrive3.set(ControlMode.Follower, this.rightDrive.getDeviceID());
		this.shooter2.follow(this.shooter1);

		this.rightDrive.setNeutralMode(NeutralMode.Brake);
		this.leftDrive.setNeutralMode(NeutralMode.Brake);
		
		this.leftDrive.setInverted(true);
		this.leftDrive2.setInverted(true);
		this.leftDrive3.setInverted(true);

		//this.rightDrive.setSensorPhase(true);
		//this.leftDrive.setSensorPhase(true);
		/*
		this.leftDrive.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive.configContinuousCurrentLimit(this.currentlimit);
		this.leftDrive2.configContinuousCurrentLimit(this.currentlimit);
		this.leftDrive3.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive2.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive3.configContinuousCurrentLimit(this.currentlimit);
		*/

		f_pidController=feeder.getPIDController();
		s_pidController=shooter1.getPIDController();
		r_pidController=revolver.getPIDController();

		f_encoder=feeder.getEncoder();
		s_encoder=shooter1.getEncoder();
		r_encoder=revolver.getEncoder();
		
		setUpPIDController(s_pidController,s_kp,s_ki,s_kd,s_kiz,s_kff,s_kMaxOutput,s_kMinOutput);
		setUpPIDController(r_pidController,r_kp,r_ki,r_kd,r_kiz,r_kff,r_kMaxOutput,r_kMinOutput);
		setUpPIDController(f_pidController,f_kp,f_ki,f_kd,f_kiz,f_kff,f_kMaxOutput,f_kMinOutput);




	}

	

	void setUpPIDController(CANPIDController pidController, double kp, double ki, double kd, double kiz, double kff,
			double kMax, double kMin) {
		pidController.setP(kp);
		pidController.setI(ki);
		pidController.setD(kd);
		pidController.setIZone(kiz);
		pidController.setFF(kff);
		pidController.setOutputRange(kMin, kMax);

		
	}
	
}
	
