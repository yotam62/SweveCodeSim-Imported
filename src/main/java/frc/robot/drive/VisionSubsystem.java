package frc.robot.drive;

import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
   //Field2d field = new Field2d();



    private Vision vision;
    private Drive drive;
    boolean disable_all_cameras = false;
        
    double lastUsedTimestamp = -1000;
    private double minTranslation = 10000.0;   
    double velocity_of_servo = 0.0;
    public boolean ruin = false;
    public record VisionMeasurement(Pose2d pose, double rotationDegreees, double timestamp, double[] std, int numTags, double avgDistance) {}
    ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();


    boolean wasDisconnected_LL4 = false;
    boolean wasDisconnected_LL3GS = false;
    boolean wasDisconnected_LL3GF = false;
    boolean disable_other_cameras = false;
 

            
    public VisionSubsystem(Vision vision, Drive drive) {
                    this.vision = vision;
                    this.drive = drive;
                   //SmartDashboard.putData("field", field);
                   getWebcamFeed(); //begins sending webcam video footage to smart dashboard
                    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
                    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-threegs", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
                    //  LimelightHelpers.SetFiducialIDFiltersOverride("limelight-threegf", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32}); TODO: add limelight helpers
    }


    public void getWebcamFeed() { //puts the webcam connected to dev port 0 onto the dashboard
      //  CameraServer.startAutomaticCapture(0);
    }
    
    

    
    @Override
    public void periodic() {
        vision.periodic();
       
        
        if (!wasDisconnected_LL4 &&!vision.isConnected_LL4) {
            Robot.reportDisconnection("Limelight 4");
            wasDisconnected_LL4 = true;
        }

        if (!wasDisconnected_LL3GS && !vision.isConnected_LL3GS) {
            Robot.reportDisconnection("Limelight 3GS");
            wasDisconnected_LL3GS = true;
    
        }

        if (!wasDisconnected_LL3GF && !vision.isConnected_LL3GF) {
            Robot.reportDisconnection("Limelight 3GF");
            wasDisconnected_LL3GF = true;
        }

        if (wasDisconnected_LL3GF && vision.isConnected_LL3GF) {
            Robot.removeDisconnection("Limelight 3GF");
            wasDisconnected_LL3GF = false;
        }

        if (wasDisconnected_LL3GS && vision.isConnected_LL3GS) {
            Robot.removeDisconnection("Limelight 3GS");
            wasDisconnected_LL3GS = false;
        }

        if (wasDisconnected_LL4 && vision.isConnected_LL4) {
            Robot.removeDisconnection("Limelight 4");
            wasDisconnected_LL4 = false;
        }


        if (vision.isNew_LL4 && vision.isConnected_LL4 && vision.tagCount_LL4 > 0 && Robot.useLimelightFour) { 
            SmartDashboard.putBoolean("sensor receiving data", true);
            double std_LL4 = (vision.avgDistance_LL4 * 0.02 ) / vision.tagCount_LL4;
            SmartDashboard.putNumber("std_LL4", std_LL4);
            double[] stds_LL4 = {std_LL4, std_LL4};
            if (std_LL4 < 0.1) {
               visionMeasurements.add(new VisionMeasurement(vision.MT2pose_LL4, vision.rotation_LL4, vision.time_LL4, stds_LL4, vision.tagCount_LL4, vision.avgDistance_LL4));
            }
        }

        // else {
        //     SmartDashboard.putBoolean("sensor receiving data", false);

        // }
        if (RobotState.isDisabled()) {}
 
         if (vision.isNew_LL3GS && vision.isConnected_LL3GS && vision.tagCount_LL3GS > 0 && !disable_other_cameras && Robot.useLimelightThreeGS) {
            double std_LL3GS = (vision.avgDistance_LL3GS * 0.02 ) / vision.tagCount_LL3GS;
            double[] stds_LL3GS = {std_LL3GS, std_LL3GS};
            if (std_LL3GS < 0.1) {
               visionMeasurements.add(new VisionMeasurement(vision.MT2pose_LL3GS, vision.rotation_LL3GS, vision.time_LL3GS, stds_LL3GS, vision.tagCount_LL3GS, vision.avgDistance_LL3GS));
            }
        }

        if (vision.isNew_LL3GF && vision.isConnected_LL3GF && vision.tagCount_LL3GF > 0 && !disable_other_cameras && Robot.useLimelightThreeGF) {
                double std_LL3GF = (vision.avgDistance_LL3GF * 0.02 ) / vision.tagCount_LL3GF;
                double[] stds_LL3GF = {std_LL3GF, std_LL3GF};
                if (std_LL3GF < 0.1) {
                visionMeasurements.add(new VisionMeasurement(Vision.MT2pose_LL3GF, vision.rotation_LL3GF, vision.time_LL3GF, stds_LL3GF, vision.tagCount_LL3GF, vision.avgDistance_LL3GF));
                }
        }

        VisionMeasurement bestmeasurement = new VisionMeasurement(new Pose2d(), 0, 0, new double[]{0,0}, 0, 0);

        if (!visionMeasurements.isEmpty()) {
            for (int i = 0; i < visionMeasurements.size(); i++) {
                if (bestmeasurement.pose().equals(new Pose2d())) {
                    bestmeasurement = visionMeasurements.get(i);
                }

                else {
                    if (bestmeasurement.std[0] > visionMeasurements.get(i).std[0]) {
                        //standard deviations are lower for this measurement, so that is the new best
                        bestmeasurement = visionMeasurements.get(i);
                    }
                }
            }
        }

        visionMeasurements.clear();

        if (!bestmeasurement.pose.equals(new Pose2d())) {
            addVisionMeasurement(bestmeasurement);
        } 
         
        }
    
        public double[] times(double multiplier, double[] list) {
            for (int i = 0; i < list.length; i++) {
                list[i] = list[i] * multiplier;
            }
            return list;
        }
    
    

        public void addVisionMeasurement(VisionMeasurement measurement) {
          //  SmartDashboard.putBoolean("ruin", ruin);

            // if (ruin) {
            // drive.addVision(new VisionMeasurement(measurement.pose().plus(new Transform2d(0.0,0.5, new Rotation2d())), measurement.rotationDegreees, measurement.timestamp, measurement.std, measurement.numTags, measurement.avgDistance));
            // }

            if (!disable_all_cameras) {
            drive.addVision(measurement); //TODO: add the drive addvision
            
            }
            


            
    }

    public void enableVision() {
        disable_all_cameras = false;

    }

    public void disableVision() {
        disable_all_cameras = true;
    }
}
