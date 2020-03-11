package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
  private RobotMap robot = new RobotMap();

  //Setpoint Shooting constants
  private double feedSpeed=5700,feedRatio=0.2, shooterPower = .7;//.7;

  //Limelight and navX constants
  private double tx_kp=0.017,tx_ki=0.00002,tx_kd=0,tx_iMax=0.02,
                ty_kp=0.016,ty_ki=0.0001,ty_kd=0,
                ahrs_kp=0.0002,ahrs_ki=0.000001,ahrs_kd=0.000001;
  private double txShoot=0,tyShoot=0,taShoot=0.1;
  private MiniPID tx_Pid=new MiniPID(tx_kp, tx_ki, tx_kd);
  private MiniPID ty_Pid=new MiniPID(ty_kp,ty_ki,ty_kd);
  private MiniPID ahrs_Pid=new MiniPID(ahrs_kp,ahrs_ki,ahrs_kd);

  //Dynamic Shooting constants
  private double visionCenterHeight=89.4;//should be 89.75 base on official guild
  private double cameraHeight=6.6,cameraAngle=31;
  private double distanceModifier=1.115,targetDistance=120;


  //shooter math
  private double shooterHeight=20,g=386.2, goalHeight=98.25, 
                shooterAngleMax=65,shooterAngleMin=30, goalDepth=29.25;
  private double shooterAngle,shooterVelocity,shooterSetPoint;
  private boolean shooterReady=false;
  private boolean usePreSetShooterPower=true;


  //Auto stuff
  public Timer autoTimer = new Timer();
  private double autoEndTime=3;
  private boolean autoShoot=true;
  private double autoTargetPosition1=20;
  private double servoPosition1=0,servoPosition2=0;
  
  
  @Override
  public void robotInit() {
    /*
    SmartDashboard.putNumber("dv_kp", robot.dv_kp);
    SmartDashboard.putNumber("dv_ki", robot.dv_ki);
    SmartDashboard.putNumber("dv_kd", robot.dv_kd);
    SmartDashboard.putNumber("dv_kiz", robot.dv_kiz);
    SmartDashboard.putNumber("dv_kff", robot.dv_kff);

    SmartDashboard.putNumber("dp_kp", robot.dp_kp);
    SmartDashboard.putNumber("dp_ki", robot.dp_ki);
    SmartDashboard.putNumber("dp_kd", robot.dp_kd);
    SmartDashboard.putNumber("dp_kiz", robot.dp_kiz);
    SmartDashboard.putNumber("dp_kff", robot.dp_kff);

    SmartDashboard.putNumber("autoTargetPosition1",autoTargetPosition1);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("s_kp", robot.s_kp);
    SmartDashboard.putNumber("s_ki", robot.s_ki);
    SmartDashboard.putNumber("s_kd", robot.s_kd);
    SmartDashboard.putNumber("s_kiz", robot.s_kiz);
    SmartDashboard.putNumber("s_kff", robot.s_kff);
    SmartDashboard.putNumber("s_kMaxOutput", robot.s_kMaxOutput);
    SmartDashboard.putNumber("s_kkMinOutput", robot.s_kMinOutput);
    SmartDashboard.putNumber("s_iAccum", robot.s_iAccum);
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

    SmartDashboard.putNumber("ahrs_kp", ahrs_kp);
    SmartDashboard.putNumber("ahrs_ki", ahrs_ki);
    SmartDashboard.putNumber("ahrs_kd", ahrs_kd);
    
    
    
    //auto
    SmartDashboard.putNumber("autoEndTime", autoEndTime);
    SmartDashboard.putBoolean("autoShoot", autoShoot);
    //Boolean setUp
    SmartDashboard.putBoolean("usePreSetShooterPower", usePreSetShooterPower);
    */

  }
  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    autoTimer.start();
    
    shooterReady=false;
    robot.setUpPIDController(robot.s_pidController, SmartDashboard.getNumber("s_kp", robot.s_kp), 
        SmartDashboard.getNumber("s_ki", robot.s_ki),SmartDashboard.getNumber("s_kd", robot.s_kd),
        SmartDashboard.getNumber("s_kiz", robot.s_kiz),SmartDashboard.getNumber("s_kff", robot.s_kff),
        SmartDashboard.getNumber("s_kMaxOutput", robot.s_kMaxOutput),SmartDashboard.getNumber("s_kMinOutput", robot.s_kMinOutput));
    robot.setUpPIDController(robot.f_pidController, SmartDashboard.getNumber("f_kp", robot.f_kp), 
        SmartDashboard.getNumber("f_ki", robot.f_ki),SmartDashboard.getNumber("f_kd", robot.f_kd),
        SmartDashboard.getNumber("f_kiz", robot.f_kiz),SmartDashboard.getNumber("f_kff", robot.f_kff),
        1,-1);
    robot.setUpPIDController(robot.r_pidController, SmartDashboard.getNumber("r_kp", robot.r_kp), 
        SmartDashboard.getNumber("r_ki", robot.r_ki),SmartDashboard.getNumber("r_kd", robot.r_kd),
        SmartDashboard.getNumber("r_kiz", robot.r_kiz),SmartDashboard.getNumber("r_kff", robot.r_kff),
        1,-1);
    robot.leftdrive_pidController_encoder.setPosition(0);
    robot.rightdrive_pidController_encoder.setPosition(0);

    //SmartDashboard.putNumber("leftEncoder", this.robot.leftdrive_pidController_encoder.getPosition());
    //SmartDashboard.putNumber("rightEncoder", this.robot.rightdrive_pidController_encoder.getPosition());
    robot.setUpPIDController(robot.rightdrive_pidController, SmartDashboard.getNumber("dp_kp", robot.dp_kp), 
      SmartDashboard.getNumber("dp_ki", robot.dp_ki),SmartDashboard.getNumber("dp_kd", robot.dp_kd),
      SmartDashboard.getNumber("dp_kiz", robot.dp_kiz),SmartDashboard.getNumber("dp_kff", robot.dp_kff),
      0.8, -0.8);

    robot.setUpPIDController(robot.leftdrive_pidController, SmartDashboard.getNumber("dp_kp", robot.dp_kp), 
      SmartDashboard.getNumber("dp_ki", robot.dp_ki),SmartDashboard.getNumber("dp_kd", robot.dp_kd),
      SmartDashboard.getNumber("dp_kiz", robot.dp_kiz),SmartDashboard.getNumber("dp_kff", robot.dp_kff),
      0.8, -0.8);
    
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

    ahrs_kp=SmartDashboard.getNumber("ahrs_kp", ahrs_kp);
    ahrs_ki=SmartDashboard.getNumber("ahrs_ki", ahrs_ki);
    ahrs_kd=SmartDashboard.getNumber("ahrs_kd", ahrs_kd);
    ahrs_Pid.reset();
    ahrs_Pid.setPID(ahrs_kp, ahrs_ki, ahrs_kd);
    ahrs_Pid.setOutputLimits(0.5);

    robot.s_iAccum=SmartDashboard.getNumber("s_iAccum", robot.s_iAccum);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    autoTimer.reset();
    autoTimer.start();
    autoEndTime=SmartDashboard.getNumber("autoEndTime", autoEndTime);
    autoShoot=SmartDashboard.getBoolean("autoShoot", autoShoot);
    shooterReady=false;

    autoTargetPosition1=SmartDashboard.getNumber("autoTargetPosition1", autoTargetPosition1);
    usePreSetShooterPower=SmartDashboard.getBoolean("usePreSetShooterPower", usePreSetShooterPower);
    //trajectory
    /*
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Waypoint[] points = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(trajectory).modify(0.5);

        // Do something with the new Trajectories...
        Trajectory left = modifier.getLeftTrajectory();
        Trajectory right = modifier.getRightTrajectory();
      for (int i = 0; i < trajectory.length(); i++) {
         Trajectory.Segment seg = trajectory.get(i);
         
         System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
          seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
            seg.acceleration, seg.jerk, seg.heading);
      }
    */
    autoTimer.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /*
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
    SmartDashboard.putNumber("distance", distance);*/
    /*while(autoTimer.get() < 5){
      robot.leftdrive_pidController.setReference(autoTargetPosition1, ControlType.kPosition);
      robot.rightdrive_pidController.setReference(autoTargetPosition1, ControlType.kPosition);
    }*/
    //SmartDashboard.putNumber("leftEncoder", this.robot.leftdrive_pidController_encoder.getPosition());
    //SmartDashboard.putNumber("rightEncoder", this.robot.rightdrive_pidController_encoder.getPosition());
    while(autoTimer.get() < 4){
      SmartDashboard.putNumber("Timer", autoTimer.get());

      if(shooterReady){
        this.robot.f_pidController.setReference(-feedSpeed,ControlType.kVelocity);
      }
      else{
        this.robot.f_pidController.setReference(0,ControlType.kVelocity);
      }
      this.robot.r_pidController.setReference(feedSpeed * feedRatio,ControlType.kVelocity);
      this.robot.s_pidController.setReference(-robot.maxRPM*this.shooterPower, ControlType.kVelocity);
      if(Math.abs(-robot.s_encoder.getVelocity()-robot.maxRPM*this.shooterPower) < 250){
        shooterReady=true;
      }
      else{
        shooterReady=false;
      }
    }
    this.robot.f_pidController.setReference(0,ControlType.kVelocity);
    this.robot.r_pidController.setReference(0,ControlType.kVelocity);
    this.robot.s_pidController.setReference(0, ControlType.kVelocity);

    /*
    if (shootingPositionA==false){
      this.robot.leftdrive_pidController.setReference(robot.dv_maxRPM*-visionTurnPower, ControlType.kVelocity);
      this.robot.rightdrive_pidController.setReference(robot.dv_maxRPM*visionTurnPower, ControlType.kVelocity);
      if (getError(0, tx)<1){
        shootingPositionA=true;
        this.robot.leftDrive.set(0);
        this.robot.rightDrive.set(0);
      }
    }else */
    /*
    if (autoShoot){
      this.robot.r_pidController.setReference(feedSpeed*.5,ControlType.kVelocity);
      this.robot.s_pidController.setReference(-robot.s_maxRPM*this.shooterPower, ControlType.kVelocity);
      if (getError(-robot.s_encoder.getVelocity(), robot.s_maxRPM*this.shooterPower)<250){
        shooterReady=true;

      }*/
      /*
      if(shooterReady&&autoTimer.get()>3){
        this.robot.f_pidController.setReference(-feedSpeed*feedRatio,ControlType.kVelocity);
      }
      */

      /*
      if(shooterReady){
        Timer.delay(2.5);
        this.robot.f_pidController.setReference(-feedSpeed*feedRatio,ControlType.kVelocity);
      }
      if(autoTimer.get()>8){
        shooterReady=false;
        this.robot.r_pidController.setReference(0,ControlType.kVelocity);
        this.robot.s_pidController.setReference(0, ControlType.kVelocity);
        this.robot.f_pidController.setReference(0,ControlType.kVelocity);
      }
    while (autoTimer.get()>autoEndTime){
      //stop
    }
    SmartDashboard.putBoolean("shootingPositionA", shootingPositionA);
    SmartDashboard.putBoolean("shooterReady", shooterReady);
    SmartDashboard.putNumber("shootTimer", shootTimer.get());
    SmartDashboard.putNumber("autoTimer", autoTimer.get());
    
    }
    */

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // Drive train PID
    robot.setUpPIDController(robot.s_pidController, SmartDashboard.getNumber("s_kp", robot.s_kp), 
        SmartDashboard.getNumber("s_ki", robot.s_ki),SmartDashboard.getNumber("s_kd", robot.s_kd),
        SmartDashboard.getNumber("s_kiz", robot.s_kiz),SmartDashboard.getNumber("s_kff", robot.s_kff),
        SmartDashboard.getNumber("s_kMaxOutput", robot.s_kMaxOutput),SmartDashboard.getNumber("s_kMinOutput", robot.s_kMinOutput));
    robot.setUpPIDController(robot.f_pidController, SmartDashboard.getNumber("f_kp", robot.f_kp), 
        SmartDashboard.getNumber("f_ki", robot.f_ki),SmartDashboard.getNumber("f_kd", robot.f_kd),
        SmartDashboard.getNumber("f_kiz", robot.f_kiz),SmartDashboard.getNumber("f_kff", robot.f_kff),
        1,-1);
    robot.setUpPIDController(robot.r_pidController, SmartDashboard.getNumber("r_kp", robot.r_kp), 
        SmartDashboard.getNumber("r_ki", robot.r_ki),SmartDashboard.getNumber("r_kd", robot.r_kd),
        SmartDashboard.getNumber("r_kiz", robot.r_kiz),SmartDashboard.getNumber("r_kff", robot.r_kff),
        1,-1);
    
        
    robot.setUpPIDController(robot.rightdrive_pidController, SmartDashboard.getNumber("dv_kp", robot.dv_kp), 
        SmartDashboard.getNumber("dv_ki", robot.dv_ki),SmartDashboard.getNumber("dv_kd", robot.dv_kd),
        SmartDashboard.getNumber("dv_kiz", robot.dv_kiz),SmartDashboard.getNumber("dv_kff", robot.dv_kff),
        1, -1);

    robot.setUpPIDController(robot.leftdrive_pidController, SmartDashboard.getNumber("dv_kp", robot.dv_kp), 
      SmartDashboard.getNumber("dv_ki", robot.dv_ki),SmartDashboard.getNumber("dv_kd", robot.dv_kd),
      SmartDashboard.getNumber("dv_kiz", robot.dv_kiz),SmartDashboard.getNumber("dv_kff", robot.dv_kff),
      1, -1);

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

    robot.s_iAccum=SmartDashboard.getNumber("s_iAccum", robot.s_iAccum);
    usePreSetShooterPower=SmartDashboard.getBoolean("usePreSetShooterPower", usePreSetShooterPower);
    robot.s_pidController.setIMaxAccum(robot.s_iAccum, 0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    shooterReady=false;
    robot.leftdrive_pidController_encoder.setPosition(0);
    robot.rightdrive_pidController_encoder.setPosition(0);
  }
  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
      // Calculate drive power
      double forward;
      if(Math.abs(this.robot.driver.getY(Hand.kLeft)) > .075){
        forward = this.robot.driver.getY(Hand.kLeft) * .8;
      }
      else{
        forward = 0;
      }
      double turn;
      if(Math.abs(this.robot.driver.getX(Hand.kRight)) > .075){
        turn = this.robot.driver.getX(Hand.kRight)*.6;
      }
      else{
        turn = 0;
      }
      /*SmartDashboard.putNumber("forward",forward);
      SmartDashboard.putNumber("turn", turn);
      SmartDashboard.putNumber("left drive RPM", this.robot.leftdrive_pidController_encoder.getVelocity());
      SmartDashboard.putNumber("right drive RPM", this.robot.rightdrive_pidController_encoder.getVelocity());*/
      
      if(this.robot.operator.getBButtonReleased()){
        shooterReady=false;
      }
      if(shooterReady){
        this.robot.f_pidController.setReference(-feedSpeed,ControlType.kVelocity);
      }
      else{
        this.robot.f_pidController.setReference(0,ControlType.kVelocity);
      }
      if(this.robot.operator.getBButton()){
        this.robot.r_pidController.setReference(feedSpeed * feedRatio,ControlType.kVelocity);
        if (usePreSetShooterPower){
          this.robot.s_pidController.setReference(-robot.maxRPM*this.shooterPower, ControlType.kVelocity);
          if(Math.abs(-robot.s_encoder.getVelocity()-robot.maxRPM*this.shooterPower) < 250){
            shooterReady=true;
          }
        }else{
          this.robot.s_pidController.setReference(-shooterSetPoint, ControlType.kVelocity);
          if(Math.abs(-robot.s_encoder.getVelocity()-shooterSetPoint) < 250){
            shooterReady=true;
          }
        }
        if(Math.abs(-robot.s_encoder.getVelocity()-robot.maxRPM*this.shooterPower) < 250){
          shooterReady=true;
        }
      }
      else{
        this.robot.s_pidController.setReference(0, ControlType.kVelocity);
      }
      if(!this.robot.operator.getBButton() && !this.robot.operator.getAButton()){
        this.robot.r_pidController.setReference(800 - 800 * this.robot.operator.getY(Hand.kLeft), ControlType.kVelocity);
        //SmartDashboard.putNumber("Revolver Setpoint", 800 - 800 * this.robot.operator.getY(Hand.kLeft));
      }
      if(this.robot.operator.getAButton() && !this.robot.operator.getBButton()){
        this.robot.r_pidController.setReference(-800, ControlType.kVelocity);
      }
      if(this.robot.operator.getXButton() && !this.robot.operator.getBButton()){
        this.robot.f_pidController.setReference(200, ControlType.kVelocity);
      }
      else if(!this.robot.operator.getBButton()){
        this.robot.f_pidController.setReference(0, ControlType.kVelocity);
      }
      /*
      double intakerPower=0;
      intakerPower=robot.operator.getTriggerAxis(Hand.kLeft)-robot.operator.getTriggerAxis(Hand.kRight);
      robot.intaker.set(intakerPower);
      */
      SmartDashboard.putNumber("Feeder RPM", this.robot.f_encoder.getVelocity());
      SmartDashboard.putNumber("Shooter RPM", this.robot.s_encoder.getVelocity());
      SmartDashboard.putNumber("Revolver RPM", this.robot.r_encoder.getVelocity());

   

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
    
    double tv = robot.limelight.getEntry("tv").getDouble(0);
		double tx = robot.limelight.getEntry("tx").getDouble(0);
		double ty = robot.limelight.getEntry("ty").getDouble(0);
    double ta = robot.limelight.getEntry("ta").getDouble(0);
    double distance=(visionCenterHeight-cameraHeight)/Math.tan(Math.toRadians(cameraAngle+ty))*distanceModifier;
    double visionTurnPower=tx_Pid.getOutput(tx);
    double visionMovePower=ty_Pid.getOutput(distance);
    

    /*SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("visionTurnPower", visionTurnPower);
    SmartDashboard.putNumber("visionMovePower", visionMovePower);
    SmartDashboard.putNumber("targetDistance", targetDistance);
    SmartDashboard.putNumber("distance", distance);*/
    double leftPow = (forward+turn) * robot.maxRPM*0.85;
    double rightPow = (forward-turn) * robot.maxRPM*0.85;
    if (robot.driver.getXButton()&&tv==1){
      //just changed the direction at 2:48
      this.robot.leftdrive_pidController.setReference(robot.maxRPM*-visionTurnPower + leftPow, ControlType.kVelocity);
      this.robot.rightdrive_pidController.setReference(robot.maxRPM*visionTurnPower + rightPow, ControlType.kVelocity);

    }else if (robot.driver.getYButton()&&tv==1){
      this.robot.leftdrive_pidController.setReference(robot.maxRPM*(visionMovePower+visionTurnPower), ControlType.kVelocity);
      this.robot.rightdrive_pidController.setReference(robot.maxRPM*(visionMovePower-visionTurnPower), ControlType.kVelocity);
    }else{
      this.robot.leftdrive_pidController.setReference(leftPow, ControlType.kVelocity);
    this.robot.rightdrive_pidController.setReference(rightPow, ControlType.kVelocity);
    }
    if (Math.abs(this.robot.leftdrive_pidController_encoder.getVelocity())<1500&&Math.abs(this.robot.rightdrive_pidController_encoder.getVelocity())<1500){
      //Low gear
      robot.shifter.set(Value.kReverse);
      SmartDashboard.putString("gear", "Low");
    }else if(Math.abs(this.robot.leftdrive_pidController_encoder.getVelocity())>2500&&Math.abs(this.robot.rightdrive_pidController_encoder.getVelocity())>2500){
      //High gear
      robot.shifter.set(Value.kForward);
      SmartDashboard.putString("gear", "High");
    }
    
    if(leftPow > 0){
      //leftPow = Math.pow(leftPow, 2);
    }
    else{
      //leftPow = -Math.pow(leftPow, 2);
    }
    if(rightPow > 0){
      //rightPow = Math.pow(rightPow, 2);
    }
    else{
      //rightPow = -Math.pow(rightPow, 2);
    }
    if (robot.operator.getBumper(Hand.kLeft)){
      robot.intakeExtention.set(Value.kReverse);
    }
    if (robot.operator.getBumper(Hand.kRight)){
      robot.intakeExtention.set(Value.kForward);
    }

    

    
    //SmartDashboard.putNumber("left Setpoint", (forward+turn) * robot.dv_maxRPM);
    //SmartDashboard.putNumber("right Setpoint", (forward-turn) * robot.dv_maxRPM);
    
    if(robot.driver.getXButtonPressed()){
        tx_Pid.reset();
        ty_Pid.reset();
      }
      shooterAngle=Math.tanh(2*(goalHeight-shooterHeight)/(distance+goalDepth));
      if(Math.toDegrees(shooterAngle)<shooterAngleMin){
        shooterAngle=shooterAngleMin;
        shooterVelocity=Math.sqrt(1/(Math.cos(shooterAngleMin)/(-1/2*g)*((goalHeight-shooterHeight)-Math.tan(shooterAngleMin)*distance)));
      }else if(Math.toDegrees(shooterAngle)>shooterAngleMax){
        shooterAngle=shooterAngleMax;
        shooterVelocity=Math.sqrt(1/(Math.cos(shooterAngleMax)/(-1/2*g)*((goalHeight-shooterHeight)-Math.tan(shooterAngleMax)*distance)));
      }else{
        shooterVelocity=Math.sqrt(g*(goalDepth+distance)*Math.cos(shooterAngle));
      }
      shooterSetPoint=60*shooterVelocity/(2*Math.PI);
      
      SmartDashboard.putNumber("shooterAngle",Math.toDegrees(shooterAngle));
      SmartDashboard.putNumber("shooterVelocity", shooterVelocity);
      SmartDashboard.putNumber("shooterSetPoint", shooterSetPoint);
      SmartDashboard.putBoolean("shooterReady", shooterReady);
      //SmartDashboard.putNumber("shooterReadyNumber", Math.abs(-robot.s_encoder.getVelocity()-robot.s_maxRPM*this.shooterPower));
      //SmartDashboard.putNumber("leftEncoder", this.robot.leftdrive_pidController_encoder.getPosition());
      //SmartDashboard.putNumber("rightEncoder", this.robot.rightdrive_pidController_encoder.getPosition());
  }
  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {
    this.teleopInit();
    SmartDashboard.putNumber("servoPosition1", servoPosition1);
    SmartDashboard.putNumber("servoPosition2", servoPosition2);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    servoPosition1=SmartDashboard.getNumber("servoPosition1", servoPosition1);
    servoPosition2=SmartDashboard.getNumber("servoPosition2", servoPosition2);
    robot.shooterServo1.setPosition(servoPosition1);
    robot.shooterServo2.setPosition(servoPosition2);

  }


  private double getError(double A,double B){
		return(Math.abs(A-B));
	}
}
