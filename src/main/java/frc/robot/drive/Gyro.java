package frc.robot.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.smartdashboard.SmartDashboard;
import java.util.Queue;

public class Gyro {

       private final Pigeon2 pigeon =
      new Pigeon2(13, "Drivetrain");

  //private final AHRS navx = new AHRS(NavXComType.kMXP_SPI, 200);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();

  public Gyro() {
    

    pigeon.getConfigurator().setYaw(180);
   yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
   yawVelocity.setUpdateFrequency(50.0);
   pitch.setUpdateFrequency(50);
   roll.setUpdateFrequency(50);
   pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
   yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
    //yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navx::getYaw);
  }

    public static boolean connected = false;
    public static  Rotation2d yawPosition = new Rotation2d();
    public static double rollDegrees = 0.0;
    public static double pitchDegrees = 0.0;
    public static double yawVelocityRadPerSec = 0.0;
    public static double[] odometryYawTimestamps = new double[] {};
    public static Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public static double odometryaccelXpositions = 0.0;
    public static double odometryaccelYpositions = 0.0;


  public void periodic() {
    SmartDashboard.putNumber("RAW", pigeon.getYaw().getValueAsDouble()); //puts the raw gyro value on smartdashboars
    connected = BaseStatusSignal.refreshAll(yaw, yawVelocity, roll, pitch).equals(StatusCode.OK); //checks if connected
    yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble()); //gets the yaw (actually important part) as a double
    yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble()); // gets the yaw velocity in rad/s
    rollDegrees = roll.getValueAsDouble(); 
    pitchDegrees = pitch.getValueAsDouble();// getting the roll and pitch (booooo)
    

    odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray(); // makes an array with the timestamps of yaw
    odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value)) // makes an array with yaw positions
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();// clearing the statussignal arrays for the next loop
    
} 
}
