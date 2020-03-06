package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotMap {

	
	public CANSparkMax leftDrive, rightDrive, leftDrive2, rightDrive2, shooter1,shooter2,feeder,revolver,intaker;
	public CANPIDController s_pidController;
	public CANPIDController f_pidController;
	public CANPIDController r_pidController;
	public CANEncoder s_encoder;
	public CANEncoder f_encoder;
	public CANEncoder r_encoder;

	public CANPIDController leftdrive_pidController;
	public CANEncoder leftdrive_pidController_encoder;
	public Servo shooterServo1, shooterServo2;
	public DoubleSolenoid shifter, pto, intakeExtention;
	//public AHRS ahrs;


	public CANPIDController rightdrive_pidController;
	public CANEncoder rightdrive_pidController_encoder;
	private static final int shooter1Port = 5, shooter2Port = 8,feederPort =7,revolverPort =9, intakerPort =6,
						leftDrive1Port = 1, leftDrive2Port = 2, rightDrive1Port = 3, rightDrive2Port = 4;
	private static final int shooterServo1Port=1,shooterServo2Port=2, shifterPort1=2,shifterPort2=3,
						ptoPort1=4,ptoPort2=5,intakeExtentionPort1=0,intakeExtentionPort2=1;
	
	public double gearRatioLow=14.88,gearRatioHigh=6.55;
	public double wheelRotationToInch=Math.PI*6;
	
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
	public double dv_kp=.000005, dv_ki=.0000005, dv_kd=.000001, dv_kiz=500, dv_kff=.000175, dv_maxRPM=5700;
	public double dp_kp=.0018, dp_ki=.000002, dp_kd=.000001, dp_kiz=500, dp_kff=.000175, dp_maxRPM=5700; 
	public double s_kp=0.00010,s_ki=0.0000001,s_kd=0,s_kiz=400,s_kff=0.000175,s_kMaxOutput=0,s_kMinOutput=-1,s_maxRPM=5700, s_iAccum=0;
	public double f_kp=0.00015,f_ki=0.000001,f_kd=0,f_kiz=2000,f_kff=0,f_kMaxOutput=1,f_kMinOutput=-1,f_maxRPM=5700;
	public double r_kp=0.00025,r_ki=0.000001,r_kd=0,r_kiz=2000,r_kff=0,r_kMaxOutput=1,r_kMinOutput=-1,r_maxRPM=5700;

	public double limelight2MaxY=24.85,limelight2MaxX=29.8;

	public final XboxController driver = new XboxController(0), operator = new XboxController(1);

	public NetworkTable limelight;

	RobotMap() {
		this.leftDrive = new CANSparkMax(leftDrive1Port, MotorType.kBrushless);
		this.leftDrive2 = new CANSparkMax(leftDrive2Port, MotorType.kBrushless);
		this.rightDrive = new CANSparkMax(rightDrive1Port, MotorType.kBrushless);
		this.rightDrive2 = new CANSparkMax(rightDrive2Port, MotorType.kBrushless);
		
		this.shooter1=new CANSparkMax(shooter1Port, MotorType.kBrushless);
		this.shooter2=new CANSparkMax(shooter2Port, MotorType.kBrushless);
		this.feeder=new CANSparkMax(feederPort, MotorType.kBrushless);
		this.revolver=new CANSparkMax(revolverPort, MotorType.kBrushless);
		//this.intaker=new CANSparkMax(intakerPort,MotorType.kBrushless);

		this.leftDrive2.follow(this.leftDrive);
		this.rightDrive2.follow(this.rightDrive);
		this.shooter2.follow(this.shooter1, true);
		this.rightDrive.setInverted(true);


		rightdrive_pidController=rightDrive.getPIDController();
		leftdrive_pidController=leftDrive.getPIDController();
		rightdrive_pidController_encoder=rightDrive.getEncoder();
		leftdrive_pidController_encoder=leftDrive.getEncoder();

		setUpPIDController(rightdrive_pidController,dv_kp,dv_ki,dv_kd,dv_kiz,dv_kff,1,-1);
		setUpPIDController(leftdrive_pidController,dv_kp,dv_ki,dv_kd,dv_kiz,dv_kff,1,-1);

		f_pidController=feeder.getPIDController();
		s_pidController=shooter1.getPIDController();
		r_pidController=revolver.getPIDController();

		f_encoder=feeder.getEncoder();
		s_encoder=shooter1.getEncoder();
		r_encoder=revolver.getEncoder();

		s_pidController.setIMaxAccum(this.s_iAccum, 0);
		
		
		setUpPIDController(s_pidController,s_kp,s_ki,s_kd,s_kiz,s_kff,s_kMaxOutput,s_kMinOutput);
		setUpPIDController(r_pidController,r_kp,r_ki,r_kd,r_kiz,r_kff,r_kMaxOutput,r_kMinOutput);
		setUpPIDController(f_pidController,f_kp,f_ki,f_kd,f_kiz,f_kff,f_kMaxOutput,f_kMinOutput);
		
		this.limelight = NetworkTableInstance.getDefault().getTable("limelight");

		this.shooterServo1=new Servo(shooterServo1Port);
		this.shooterServo2=new Servo(shooterServo2Port);
		

		shifter=new DoubleSolenoid(shifterPort1,shifterPort2);
		pto=new DoubleSolenoid(ptoPort1,ptoPort2);
		intakeExtention=new DoubleSolenoid(intakeExtentionPort1,intakeExtentionPort2);
		//new AHRS(SPI.Port.kMXP);
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
	double getError(double A,double B){
		return(Math.abs(A-B));
	}
	
}
	
