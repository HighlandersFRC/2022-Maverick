package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.ClimbSequenceP2;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.FaceTarget;
import frc.robot.commands.FireBalls;
import frc.robot.commands.FireBallsNoVision;
import frc.robot.commands.FireOneBall;
import frc.robot.commands.HoldRotatingArmOnBar;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LoadedRobotClimb;
import frc.robot.commands.LockDriveWheels;
import frc.robot.commands.Outtake;
import frc.robot.commands.PositionRotatingClimber;
import frc.robot.commands.PositionVerticalClimber;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.RunRotatingClimber;
import frc.robot.commands.RunVerticalClimber;
import frc.robot.commands.TurnDriveTrain;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.commands.autos.FiveBallAuto;
import frc.robot.commands.autos.OneBallAuto;
import frc.robot.commands.autos.TestAuto;
import frc.robot.commands.autos.ThreeBallAuto;
import frc.robot.commands.autos.TwoBallAuto;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.MqttPublish;
import frc.robot.subsystems.MqttSubscribe;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lights.LEDMode;
import frc.robot.tools.PneumaticsControl;
import frc.robot.tools.ShotAdjuster;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.System.Logger.Level;

import javax.swing.text.Position;

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

  private MqttPublish publish = new MqttPublish();
  private MqttSubscribe subscribe = new MqttSubscribe();

  private final Shooter shooter = new Shooter();
  private final Hood hood  = new Hood();

  private final Climber climber = new Climber(pneumatics);
  private final Carriage carriage = new Carriage(pneumatics);

  private Lights lights = new Lights();

  private final Peripherals peripherals = new Peripherals(subscribe, lights);
  private final Drive drive = new Drive(peripherals, publish);

  private MagIntake magIntake = new MagIntake(pneumatics);

  private ThreeBallAuto threeBallAuto = new ThreeBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private OneBallAuto oneBallAuto = new OneBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private TwoBallAuto twoBallAuto = new TwoBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private FiveBallAuto fiveBallAuto = new FiveBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private TestAuto testAuto = new TestAuto(drive);

  private UsbCamera cameraBack;
  private UsbCamera cameraFront;
  private VideoSink server;

  private ShotAdjuster shotAdjuster = new ShotAdjuster();
  private Button whenPressed;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    cameraBack = CameraServer.startAutomaticCapture("Back Cam", "/dev/video0");
    cameraBack.setResolution(320, 240);
    cameraBack.setFPS(15);
    SmartDashboard.putNumber("Lights", 0);

    cameraFront = CameraServer.startAutomaticCapture("Front Cam", "/dev/video1");
    cameraFront.setResolution(320, 240);
    cameraFront.setFPS(15);

    server = CameraServer.addSwitchedCamera("driverVisionCameras");  
    server.setSource(cameraFront);
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
        // System.out.println(pathJSON);
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
        // System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is3BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/5BallPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        // System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.is5BallAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/5BallPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        // System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.isTestAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/TestNoTurn.json");
        FileReader scanner = new FileReader(pathingFile);
        pathJSON = new JSONArray(new JSONTokener(scanner));
        // System.out.println(pathJSON);
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
        // System.out.println(pathJSON);
      }
      catch(Exception e) {
        // System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    // System.out.println(pathJSON);
    drive.init();
    carriage.init();
  }

  @Override
  public void robotPeriodic() {
    lights.periodic();
    SmartDashboard.putNumber("Lime Light X", peripherals.getLimeLightX());
    SmartDashboard.putNumber("Lime Light Y", peripherals.getLimeLightY());
    // if(!magIntake.getUpperBeamBreak()) {
    //   lights.setMode(LEDMode.RAINBOW);
    // }
    //System.out.println(peripherals.getVisionArray());
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("LIMIT SWITCH", hood.getLowerLimitSwitch());
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("HOOD", hood.getHoodPosition());
    SmartDashboard.putNumber("RPM ADJUSTMENT", shotAdjuster.getRPMAdjustment());
    SmartDashboard.putNumber("HOOD ADJUSTMENT", shotAdjuster.getHoodAdjustment());

    SmartDashboard.putBoolean("BOTTOM BEAM BREAK", magIntake.getLowerBackBeamBreak());
    SmartDashboard.putBoolean("UPPER BEAM BREAK", magIntake.getUpperBeamBreak());

    // climber.postRotatingClimberEncoder();

    SmartDashboard.putNumber("VClimber", carriage.getclimberFalcon1Position());
    SmartDashboard.putNumber("RCLIMBER", climber.getRotatingMotorPosition());

    SmartDashboard.putBoolean("CLIMB LIMIT SWITCH", climber.getLimitSwitch());
    SmartDashboard.putNumber("ROLL", peripherals.getNavxPitch());

    SmartDashboard.putNumber("ROTATING TEMPERATURE", climber.getRotatingMotorTemperature());
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
    // System.out.println(pathJSON);
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
    else if(OI.isTestAuto()) {
      testAuto.schedule();
    }
    else {
      testAuto.schedule();
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

    //OI.driverA.whenPressed(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 6.25, 1400, 0.5, 0.5, shotAdjuster));
    OI.driverA.whenPressed(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 0.0, 1400, 0.75, 0.75, shotAdjuster));
    // OI.driverB.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 22.0, 1490, 0.75, 0.75, shotAdjuster, 0, true));
    OI.driverB.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 10, 1400, 0.75, 0.75, shotAdjuster, 0, true));
    OI.driverY.whenPressed(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 9.25, 1400, 0.5, 0.5, shotAdjuster));
    OI.driverX.whenPressed(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 30, 1650, 0.75, 0.75, shotAdjuster));
    // OI.driverMenuButton.whenPressed(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 23, 1530, 0.5, 0.5, shotAdjuster));
    // OI.driverMenuButton.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 24, 1560, 0.5, 0.5, shotAdjuster, 0 , false));

    OI.driverMenuButton.whenPressed(new FireOneBall(drive, magIntake, shooter, hood, peripherals, lights, 10, 1400, 0.75, 0.75, shotAdjuster, 0, true));

    // OI.operatorB.whileHeld(new LockDriveWheels(drive));

    OI.driverViewButton.whileHeld(new ZeroNavxMidMatch(drive));

    OI.operatorA.whenPressed(new LoadedRobotClimb(carriage, 0));
    OI.operatorY.whenPressed(new PositionVerticalClimber(carriage, 21));
    OI.operatorX.whenPressed(new ClimbSequence(climber, carriage));
    // OI.operatorB.whenPressed(new ClimbSequenceP2(climber, carriage));
    OI.operatorB.whenPressed(new HoldRotatingArmOnBar(climber, 30, peripherals));

    OI.operatorViewButton.whenPressed(new ResetClimber(climber, carriage));

    OI.operatorRT.whileHeld(new RunVerticalClimber(carriage, 0.15));
    OI.operatorLT.whileHeld(new RunVerticalClimber(carriage, -0.15));
    OI.operatorRB.whileHeld(new RunRotatingClimber(climber, 0.1));
    OI.operatorLB.whileHeld(new RunRotatingClimber(climber, -0.35));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Controller Y", OI.getDriverLeftY());
    SmartDashboard.putNumber("Controller X", OI.getDriverLeftX());
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());

    // if (OI.getDriverA()) {
    //   System.out.println("+");
    // }

    if(OI.getPOV() == 0) {
      shotAdjuster.increaseRPM();
    }
    else if(OI.getPOV() == 180) {
      shotAdjuster.decreaseRPM();
    }
    else {

    }

    if(OI.getPOV() == 270) {
      shotAdjuster.increaseHood();
    }
    else if(OI.getPOV() == 90) {
      shotAdjuster.decreaseHood();
    }
    else {

    }

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
