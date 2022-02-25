package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FaceTarget;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.DriveAlignedToTarget;
import frc.robot.commands.EjectBalls;
import frc.robot.commands.FireBalls;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Outtake;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.commands.autos.ThreeBallAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
// import frc.robot.subsystems.Hood;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.MqttPublish;
import frc.robot.subsystems.MqttSubscribe;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Shooter;
import frc.robot.tools.PneumaticsControl;

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
  private MagIntake magIntake = new MagIntake(pneumatics);

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

  private ThreeBallAuto threeBallAuto = new ThreeBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
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

    try {
      pathingFile = new File("/home/lvuser/deploy/Adj3Ball.json");
      FileReader scanner = new FileReader(pathingFile);
      pathJSON = new JSONArray(new JSONTokener(scanner));
      System.out.println(pathJSON);
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    drive.init();

  }

  @Override
  public void robotPeriodic() {
    // System.out.println(climber.getClimberEncoder());
    lights.periodic();
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
    // System.out.println(hood.getLowerLimitSwitch());
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
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    System.out.println(pathJSON);
    drive.autoInit(pathJSON);
    // peripherals.init();
    // testPath = new ContinuousAccelerationInterpolation(drive, pathJSON);
    // testPath.schedule();
    threeBallAuto.schedule();
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

    OI.driverA.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 5, 2200, 0.5, 0.5));
    OI.driverB.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 17, 2600, 0.75, 0.75));
    OI.driverY.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 20, 2900, 0.5, 0.5));
    OI.driverX.whenPressed(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 24, 3200, 0.5, 0.5));

    OI.driverY.whenPressed(new FaceTarget(drive));

    OI.driverViewButton.whenPressed(new ZeroNavxMidMatch(drive));

    // OI.driveStartButton.whileHeld(new DriveAlignedToTarget(drive, peripherals));

    // OI.operatorX.whileHeld(new RaiseClimber(climber));
    // OI.operatorB.whileHeld(new ClimbRobot(climber));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Controller Y", OI.getDriverLeftY());
    SmartDashboard.putNumber("Controller X", OI.getDriverLeftX());
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());

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
