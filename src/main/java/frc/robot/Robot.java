package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
  //private Joystick s_stick;
  private double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075,
  maxVelDT = 400,DTkP = 5,DTkI = 0.003, DTkD = 0, LDTkF = 1, RDTkF = 1;
  private double feedSpeed=2000,feedRatio=1.8, shooterPower = .75;

  private RobotMap robot = new RobotMap();

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

    
    SmartDashboard.putNumber("DTkP", this.DTkP);
		SmartDashboard.putNumber("DTkI", this.DTkI);
		SmartDashboard.putNumber("DTkD", this.DTkD);
		SmartDashboard.putNumber("LDTkF", this.LDTkF);
    SmartDashboard.putNumber("RDTkF", this.RDTkF);
    SmartDashboard.putNumber("Max Vel DT", this.maxVelDT);

    SmartDashboard.putNumber("feedSpeed", feedSpeed);
    SmartDashboard.putNumber("feedRatio", feedRatio);


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
    double temp;
    // Drive train PID   
		this.DTkP = SmartDashboard.getNumber("DTkP", this.DTkP);
		this.DTkI = SmartDashboard.getNumber("DTkI", this.DTkI);
		this.DTkD = SmartDashboard.getNumber("DTkD", this.DTkD);
		this.LDTkF = SmartDashboard.getNumber("LDTkF", this.LDTkF);
    this.RDTkF = SmartDashboard.getNumber("RDTkF", this.RDTkF);
    
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
        
    this.feedSpeed=SmartDashboard.getNumber("feedSpeed", feedSpeed);
    this.feedRatio=SmartDashboard.getNumber("feedRatio", feedRatio);
    
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
    SmartDashboard.putNumber("Max Vel DT", this.maxVelDT);
  }
  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

      

      // Calculate drive power
			double forward = -this.robot.driver.getY(Hand.kLeft) * .8,
      turn = this.robot.driver.getX(Hand.kRight);
      SmartDashboard.putNumber("forward",forward);
      SmartDashboard.putNumber("turn", turn);
/*
      if (Math.abs(forward) < 0.2d) {
        forward = 0;
      }
      if (Math.abs(turn) < 0.2d) {
        turn = 0;
      }
      */
      this.robot.leftDrive.set(ControlMode.PercentOutput, (forward-turn)*.4);
      this.robot.rightDrive.set(ControlMode.PercentOutput, (forward+turn)*.4);

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
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
