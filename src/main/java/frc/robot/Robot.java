package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FaceTarget;
import frc.robot.commands.CancelClimber;
import frc.robot.commands.ClimbRobot;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.DriveAlignedToTarget;
import frc.robot.commands.EjectBalls;
import frc.robot.commands.FireBalls;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LowerClimberToBar;
import frc.robot.commands.Outtake;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.commands.autos.FiveBallAuto;
import frc.robot.commands.autos.OneBallAuto;
import frc.robot.commands.autos.TestFiveBallAuto;
import frc.robot.commands.autos.ThreeBallAuto;
import frc.robot.commands.autos.TwoBallAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.MqttPublish;
import frc.robot.subsystems.MqttSubscribe;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.PneumaticsControl;
import frc.robot.tools.ShotAdjuster;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.json.JSONArray;
import org.json.JSONTokener;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  File f;
  BufferedWriter bw;
  FileWriter fw;
  double startTime;

  File pathingFile;
  String pathString;

  JSONArray pathJSON;

  ContinuousAccelerationInterpolation testPath;

  private final PneumaticsControl pneumatics = new PneumaticsControl();
 
  private final String subCameraTopic = "/sensors/camera";
  private final String pubCameraTopic = "/robot/camera";

  private MqttPublish publish = new MqttPublish();
  private MqttSubscribe subscribe = new MqttSubscribe();

  private final Peripherals peripherals = new Peripherals(subscribe);

  private final Drive drive = new Drive(peripherals, publish);
  private final Shooter shooter = new Shooter();
  private final Hood hood  = new Hood();

  private final Climber climber = new Climber(pneumatics);

  private Lights lights = new Lights();

  private MagIntake magIntake = new MagIntake(pneumatics);

  private ThreeBallAuto threeBallAuto = new ThreeBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private OneBallAuto oneBallAuto = new OneBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private TwoBallAuto twoBallAuto = new TwoBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private FiveBallAuto fiveBallAuto = new FiveBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private TestFiveBallAuto fiveBallP2 = new TestFiveBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private UsbCamera camera;
  private VideoSink server;

  private ShotAdjuster shotAdjuster = new ShotAdjuster();
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    camera = CameraServer.startAutomaticCapture("Vision Stream", "/dev/video0");
    camera.setResolution(320, 240);
    camera.setFPS(15);

    server = CameraServer.getServer();  
    server.setSource(camera);
    // System.out.println("###########");
    peripherals.init();
    magIntake.init();
    shooter.init();
    hood.init();
    lights.init();
    climber.init();

    // publish.publish(pubCameraTopic);
    subscribe.subscribe(subCameraTopic);
    // m_robotContainer = new RobotContainer();

    if(OI.is1BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/1BallAuto.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is2BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2BallAuto.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is3BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/Adj3Ball.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is5BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/Adj3Ball.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is5BallPt2()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/5BallPart2.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else {
      try {
        pathingFile = new File("/home/lvuser/deploy/1BallAuto.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    System.out.println(pathJSON);
    drive.init();
  }

  @Override
  public void robotPeriodic() {
    lights.periodic();
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("RPM ADJUSTMENT", shotAdjuster.getRPMAdjustment());
    SmartDashboard.putNumber("HOOD ADJUSTMENT", shotAdjuster.getHoodAdjustment());

    SmartDashboard.putBoolean("BOTTOM BEAM BREAK", magIntake.getLowerBackBeamBreak());
    SmartDashboard.putBoolean("UPPER BEAM BREAK", magIntake.getUpperBeamBreak());
    }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      if(bw != null){ 
        bw.close();
      }
      if(fw != null){
        fw.close();
      }
     
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // peripherals.turnLightRingOff();
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    climber.zeroRotatingEncoder();
    System.out.println(pathJSON);
    drive.autoInit(pathJSON);
    // peripherals.init();
    // testPath = new ContinuousAccelerationInterpolation(drive, pathJSON);
    // testPath.schedule();
    if(OI.is3BallAuto()) {
      threeBallAuto.schedule();
    }
    else if(OI.is1BallAuto()) {
      oneBallAuto.schedule();
    }
    else if(OI.is2BallAuto()) {
      twoBallAuto.schedule();
    }
    else if(OI.is5BallAuto()) {
      fiveBallAuto.schedule();
    }
    else if(OI.is5BallPt2()) {
      fiveBallP2.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    startTime = Timer.getFPGATimestamp();
    // try{
    //   f = new File("/home/lvuser/Navx/NavxValues" +  (int) startTime + ".csv");
    //   if(!f.exists()){
    //     f.createNewFile();  
    //   }
    //   fw = new FileWriter(f);
    //   bw = new BufferedWriter(fw);
    //   bw.write("started collecting \n" );
      
      
    //   }
    //   catch (Exception e){
    //     e.printStackTrace();
    //   }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    OI.driverRT.whileHeld(new IntakeBalls(magIntake, lights));
    OI.driverLT.whileHeld(new Outtake(magIntake));

    OI.driverA.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 0, 1500, 0.5, 0.5, shotAdjuster));
    OI.driverB.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 21, 2000, 0.75, 0.75, shotAdjuster));
    OI.driverY.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 25, 2200, 0.5, 0.5, shotAdjuster));
    // OI.driverX.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 24, 3200, 0.5, 0.5));

    //OI.driverY.whenPressed(new FaceTarget(drive));

    OI.driverViewButton.whileHeld(new ZeroNavxMidMatch(drive));

    

    // OI.driveStartButton.whileHeld(new DriveAlignedToTarget(drive, peripherals));

    OI.operatorY.whileHeld(new RaiseClimber(climber, -0.25));
    OI.operatorA.whileHeld(new ClimbRobot(climber, 0.5));

    OI.operatorX.whenPressed(new LowerClimberToBar(climber));

    OI.operatorViewButton.whenPressed(new CancelClimber(climber));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Controller Y", OI.getDriverLeftY());
    SmartDashboard.putNumber("Controller X", OI.getDriverLeftX());
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());

    // if (OI.operatorB.get()) {
    //   //new RaiseClimber(climber, 0.2);
    //   //climber.unlockExtendingClimber();
    //   //climber.setClimberPercents(0.6);
    //   climber.setRotatingClimberPercent(0.6);
    // } else if (OI.operatorX.get()) {
    //   //new RaiseClimber(climber, -0.2);
    //   //climber.unlockExtendingClimber();
    //   //climber.setClimberPercents(-0.6);
    //   climber.setRotatingClimberPercent(-0.6);
    // } else {
    //   //new RaiseClimber(climber, 0.0);
    //   //climber.lockExtendingClimber();
    //   //climber.setClimberPercents(0.0);
    //   climber.setRotatingClimberPercent(0.0);
    // }

    // if(OI.operatorController.getPOV(0) != -1) {
    //   shotAdjuster.increaseRPM();
    // }
    // else if(OI.operatorController.getPOV(180) != -1) {
    //   shotAdjuster.decreaseRPM();
    // }
    // else {

    // }

    // if(OI.operatorController.getPOV(270) != -1) {
    //   shotAdjuster.increaseHood();
    // }
    // else if(OI.operatorController.getPOV(90) != -1) {
    //   shotAdjuster.decreaseHood();
    // }
    // else {

    // }

    //System.out.println("RIGHT --> " + climber.getRightClimberPosition());
    //System.out.println("LEFT --> " + climber.getLeftClimberPosition());
  }

  @Override
  public void testInit() {
    OI.driverRT.whileHeld(new IntakeBalls(magIntake, lights));
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
