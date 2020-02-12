package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
  //private Joystick s_stick;

  private Timer time;
  private RobotMap robot = new RobotMap();
  private double feedSpeed=2000,feedRatio=1.8, shooterPower = .75;
  private double tx_kp=0.009,tx_ki=0.00008,tx_kd=0.005,tx_iMax=0.15,
                ty_kp=0.016,ty_ki=0.0001,ty_kd=0;
  private double txShoot=0,tyShoot=0,taShoot=0.1;
  private MiniPID tx_Pid=new MiniPID(tx_kp, tx_ki, tx_kd);
  private MiniPID ty_Pid=new MiniPID(ty_kp,ty_ki,ty_kd);
  private double visionCenterHeight=89.4;//should be 89.75 base on official guild
  private double cameraHeight=6.6,cameraAngle=31;
  private double distanceModifier=1.115,targetDistance=120;

  //shooter math
  private double shooterHeight=20,g=386.2, goalHeight=98.25, 
                shooterAngleMax=65,shooterAngleMin=30, goalDepth=29.25;
  private double shooterAngle,shooterVelocity,shooterSetPoint;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("s_kp", robot.s_kp);
    SmartDashboard.putNumber("s_ki", robot.s_ki);
    SmartDashboard.putNumber("s_kd", robot.s_kd);
    SmartDashboard.putNumber("s_kiz", robot.s_kiz);
    SmartDashboard.putNumber("s_kff", robot.s_kff);
    SmartDashboard.putNumber("s_kMaxOutput", robot.s_kMaxOutput);
    SmartDashboard.putNumber("s_kkMinOutput", robot.s_kMinOutput);
    SmartDashboard.putNumber("r_kp", robot.r_kp);
    SmartDashboard.putNumber("r_ki", robot.r_ki);
    SmartDashboard.putNumber("r_kd", robot.r_kd);
    SmartDashboard.putNumber("r_kiz", robot.r_kiz);
    SmartDashboard.putNumber("r_kff", robot.r_kff);
    SmartDashboard.putNumber("r_kMaxOutput", robot.r_kMaxOutput);
    SmartDashboard.putNumber("r_kkMinOutput", robot.r_kMinOutput);

    SmartDashboard.putNumber("f_kp", robot.f_kp);
    SmartDashboard.putNumber("f_ki", robot.f_ki);
    SmartDashboard.putNumber("f_kd", robot.f_kd);
    SmartDashboard.putNumber("f_kiz", robot.f_kiz);
    SmartDashboard.putNumber("f_kff", robot.f_kff);
    SmartDashboard.putNumber("f_kMaxOutput", robot.f_kMaxOutput);
    SmartDashboard.putNumber("f_kkMinOutput", robot.f_kMinOutput);

    SmartDashboard.putNumber("feedSpeed", feedSpeed);
    SmartDashboard.putNumber("feedRatio", feedRatio);

    SmartDashboard.putNumber("targetDistance", targetDistance);
    SmartDashboard.putNumber("taShoot",taShoot);
    SmartDashboard.putNumber("txShoot", txShoot);
    SmartDashboard.putNumber("tyShoot", tyShoot);

    SmartDashboard.putNumber("tx_kp", tx_kp);
    SmartDashboard.putNumber("tx_ki", tx_ki);
    SmartDashboard.putNumber("tx_kd", tx_kd);
    SmartDashboard.putNumber("tx_iMax", tx_iMax);

    SmartDashboard.putNumber("ty_kp", ty_kp);
    SmartDashboard.putNumber("ty_ki", ty_ki);
    SmartDashboard.putNumber("ty_kd", ty_kd);


  }
    //this.zero = 2162;
  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // Drive train PID
    /*
    robot.setUpPIDController(robot.s_pidController, SmartDashboard.getNumber("s_kp", robot.s_kp), 
        SmartDashboard.getNumber("s_ki", robot.s_ki),SmartDashboard.getNumber("s_kd", robot.s_kd),
        SmartDashboard.getNumber("s_kiz", robot.s_kiz),SmartDashboard.getNumber("s_kff", robot.s_kff),
        SmartDashboard.getNumber("s_kMaxOutput", robot.s_kMaxOutput),SmartDashboard.getNumber("s_kMinOutput", robot.s_kMinOutput));
    robot.setUpPIDController(robot.f_pidController, SmartDashboard.getNumber("f_kp", robot.f_kp), 
        SmartDashboard.getNumber("f_ki", robot.f_ki),SmartDashboard.getNumber("f_kd", robot.f_kd),
        SmartDashboard.getNumber("f_kiz", robot.f_kiz),SmartDashboard.getNumber("f_kff", robot.f_kff),
        SmartDashboard.getNumber("f_kMaxOutput", robot.f_kMaxOutput),SmartDashboard.getNumber("f_kMinOutput", robot.f_kMinOutput));
    robot.setUpPIDController(robot.r_pidController, SmartDashboard.getNumber("r_kp", robot.r_kp), 
        SmartDashboard.getNumber("r_ki", robot.r_ki),SmartDashboard.getNumber("r_kd", robot.r_kd),
        SmartDashboard.getNumber("r_kiz", robot.r_kiz),SmartDashboard.getNumber("r_kff", robot.r_kff),
        SmartDashboard.getNumber("r_kMaxOutput", robot.r_kMaxOutput),SmartDashboard.getNumber("r_kMinOutput", robot.r_kMinOutput));
    */    
    this.feedSpeed=SmartDashboard.getNumber("feedSpeed", feedSpeed);
    this.feedRatio=SmartDashboard.getNumber("feedRatio", feedRatio);

    targetDistance=SmartDashboard.getNumber("targetDistance", targetDistance);
    taShoot=SmartDashboard.getNumber("taShoot",taShoot);
    txShoot=SmartDashboard.getNumber("txShoot", txShoot);
    tyShoot=SmartDashboard.getNumber("tyShoot", tyShoot);
    tx_kp=SmartDashboard.getNumber("tx_kp", tx_kp);
    tx_ki=SmartDashboard.getNumber("tx_ki", tx_ki);
    tx_kd=SmartDashboard.getNumber("tx_kd", tx_kd);
    tx_iMax=SmartDashboard.getNumber("tx_iMax", tx_iMax);
    tx_Pid.reset();
    tx_Pid.setPID(tx_kp, tx_ki, tx_kd);
    tx_Pid.setMaxIOutput(tx_iMax);
    tx_Pid.setSetpoint(txShoot);
    tx_Pid.setOutputLimits(0.4);

    ty_kp=SmartDashboard.getNumber("ty_kp", ty_kp);
    ty_ki=SmartDashboard.getNumber("ty_ki", ty_ki);
    ty_kd=SmartDashboard.getNumber("ty_kd", ty_kd);
    ty_Pid.reset();
    ty_Pid.setPID(ty_kp, ty_ki, ty_kd);
    ty_Pid.setSetpoint(targetDistance);
    ty_Pid.setOutputLimits(0.4);
    

    
    /*
    this.robot.leftDrive.config_kP(0, this.DTkP);
		this.robot.leftDrive.config_kI(0, this.DTkI);
		this.robot.leftDrive.config_kD(0, this.DTkD);
		this.robot.leftDrive.config_kF(0, this.LDTkF);
		this.robot.rightDrive.config_kP(0, this.DTkP);
		this.robot.rightDrive.config_kI(0, this.DTkI);
		this.robot.rightDrive.config_kD(0, this.DTkD);
    this.robot.rightDrive.config_kF(0, this.RDTkF);
    */

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }
  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

      // Calculate drive power
			double forward = -this.robot.driver.getY(Hand.kLeft) * .9,
      turn = this.robot.driver.getX(Hand.kRight)*.7;
      SmartDashboard.putNumber("forward",forward);
      SmartDashboard.putNumber("turn", turn);
      
      /*
      if (this.robot.driver.getAButton()){
        this.robot.f_pidController.setReference(feedSpeed*feedRatio,ControlType.kVelocity);
      }else{
        this.robot.f_pidController.setReference(0, ControlType.kVelocity);
      }
      if (this.robot.driver.getBButton()){
        this.robot.r_pidController.setReference(feedSpeed,ControlType.kVelocity);
      }else if(this.robot.driver.getYButton()){
        this.robot.r_pidController.setReference(feedSpeed*.5,ControlType.kVelocity);
      }else{
        this.robot.r_pidController.setReference(0, ControlType.kVelocity);
      }
      this.robot.s_pidController.setReference(-robot.s_maxRPM*this.robot.driver.getTriggerAxis(Hand.kLeft)*this.shooterPower, ControlType.kVelocity);
      SmartDashboard.putNumber("Feeder RPM", this.robot.f_encoder.getVelocity());
      SmartDashboard.putNumber("Shooter RPM", this.robot.s_encoder.getVelocity());
      SmartDashboard.putNumber("Revolver RPM", this.robot.r_encoder.getVelocity());
      */
      /*
      if (this.robot.driver.getBumper(Hand.kLeft)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * this.maxVelDT);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * this.maxVelDT);
			} else if (this.robot.driver.getBumper(Hand.kRight)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn * .5) * .4 * this.maxVelDT);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn * .5) * .4 * this.maxVelDT);
			} else {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * .4 * this.maxVelDT);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * .4 * this.maxVelDT);
      }
      */

   

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    
    
    /*
    double setPoint = -1*maxRPM;
    s_pidController.setReference(setPoint, ControlType.kVelocity);
    SmartDashboard.putNumber("Output", s_motor.getAppliedOutput());
    //s_motor.set(-.5);
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", s_encoder.getVelocity());
    */

    double tv = robot.limelight.getEntry("tv").getDouble(0);
		double tx = robot.limelight.getEntry("tx").getDouble(0);
		double ty = robot.limelight.getEntry("ty").getDouble(0);
    double ta = robot.limelight.getEntry("ta").getDouble(0);
    double distance=(visionCenterHeight-cameraHeight)/Math.tan(Math.toRadians(cameraAngle+ty))*distanceModifier;
    double visionTurnPower=tx_Pid.getOutput(tx);
    double visionMovePower=ty_Pid.getOutput(distance);
    

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("visionTurnPower", visionTurnPower);
    SmartDashboard.putNumber("visionMovePower", visionMovePower);
    SmartDashboard.putNumber("targetDistance", targetDistance);
    SmartDashboard.putNumber("distance", distance);
    if(robot.driver.getAButton()&&tv==1){
      this.robot.leftDrive.set(ControlMode.PercentOutput, visionTurnPower);
      this.robot.rightDrive.set(ControlMode.PercentOutput, -visionTurnPower);
    }else if (robot.driver.getXButton()&&tv==1){
      this.robot.leftDrive.set(ControlMode.PercentOutput, -visionMovePower);
      this.robot.rightDrive.set(ControlMode.PercentOutput, -visionMovePower);
    }else if (robot.driver.getYButton()&&tv==1){
      this.robot.leftDrive.set(ControlMode.PercentOutput, -visionMovePower+visionTurnPower);
      this.robot.rightDrive.set(ControlMode.PercentOutput, -visionMovePower-visionTurnPower);
    }else{
      this.robot.leftDrive.set(ControlMode.PercentOutput, (forward-turn)*.4);
      this.robot.rightDrive.set(ControlMode.PercentOutput, (forward+turn)*.4);
    }
    if(robot.driver.getBButton()){
        tx_Pid.reset();
        ty_Pid.reset();
      }
      shooterAngle=Math.tanh(2*(goalHeight-shooterHeight)/(distance+goalDepth));
      shooterVelocity=Math.sqrt(g*(goalDepth+distance)*Math.cos(shooterAngle));
      shooterSetPoint=42*shooterVelocity/(2*Math.PI);
      SmartDashboard.putNumber("shooterAngle",Math.toDegrees(shooterAngle));
      SmartDashboard.putNumber("shooterVelocity", shooterVelocity);
      SmartDashboard.putNumber("shooterSetPoint", shooterSetPoint);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
