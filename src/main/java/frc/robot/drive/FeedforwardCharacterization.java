package frc.robot.drive;

import java.util.LinkedList;
import java.util.List;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.XboxController;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class FeedforwardCharacterization  {
    
    private static final double FF_START_DELAY = 2;
    private static final double FF_RAMP_RATE = 0.1;
    
        public static Command feedforwardCommand(Drive drive, Gamepad controller) { 
            List<Double> velocitySamples = new LinkedList<>();
            List<Double> voltageSamples = new LinkedList<>();
            Timer timer = new Timer();
    
        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),
    
            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),
    
            // Start timer
            Commands.runOnce(timer::restart),
    
            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  if (controller.getXButton()) {
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                  }
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  
                  SmartDashboard.putNumber("kS:", kS);
                  SmartDashboard.putNumber("kV:", kV);
                  

    }));
}
  }