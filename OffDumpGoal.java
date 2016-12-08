package finalBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Beth Lester on 12/1/2016.
 */

@Autonomous(name="DumpGoal", group="ElonDev")

public class OffDumpGoal extends LinearOpMode {
    FinalHardware robot = new FinalHardware();

    public boolean isRedTeam;
    public boolean isBlueTeam;

    public int threshold;
    public int minBrightness;
    public int maxBrightness;

    private double Pc = 0.5; //oscillation period
    private double Kc = 0.015; //critical gain
    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in seconds

    private double Kp = (0.6*Kc)/2; //0.6*Kc is giving about 2x our ideal Kp
    private double Ki = (2*Kp*dT)/Pc;
    private double Kd = (Kp*Pc)/(8*dT);

    private double sumError = 0.0;
    private double dError = 0.0;
    private double prevError = 0.0;

    private int loopCounter = 0;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode()throws InterruptedException {
        //initialize the robot
        robot.init(hardwareMap);

        robot.moveRobot(robot.SLOW_POWER,40);

        //sleep at end of ramp for testing
        robot.stop();

    }

//        int red = robot.sensorColor.red();
//        int blue = robot.sensorColor.blue();
//
//        if (red > blue){
//            isRedTeam = true;
//            isBlueTeam = false;
//        }
//        else if (blue > red){
//            isRedTeam = false;
//            isBlueTeam = true;
//        }
//
//        telemetry.addData("Status","Finished Init");
//        telemetry.update();
//
//
//        //while on the ramp
//        if(isRedTeam){
//            while(red <= 255 && red > 200){
//                red = robot.sensorColor.red();
//                robot.start(FinalHardware.SLOW_POWER);
//            }
//        }
//        else if(isBlueTeam){
//            while(blue <= 255 && blue > 200){
//                blue = robot.sensorColor.blue();
//                robot.start(FinalHardware.SLOW_POWER);
//            }
//        }
//
//        //wait 3 seconds for testing
//        sleep(3000);
//
//        //once the robot is off the ramp, get back to a heading of 0
//        int currentHeading = robot.sensorGyro.getHeading();
//        while (currentHeading != 0){
//            if(currentHeading > 0 && currentHeading < 180) {
//                robot.spin(FinalHardware.SLOW_POWER);
//                currentHeading = robot.sensorGyro.getHeading();
//            }
//            if(currentHeading <= 359 && currentHeading >= 180){
//                robot.spin(-FinalHardware.SLOW_POWER);
//                currentHeading = robot.sensorGyro.getHeading();
//            }
//        }
//
//        //wait 3 seconds for testing
//        sleep(3000);
//
//        //follow white line to goals
//        calibrateColors();
//
//        int brightness = robot.sensorColor.alpha();
//        int error = threshold - brightness;
//
//        sumError = 0.9*sumError + error;
//        prevError = error;
//        dError = error - prevError;
//
//        double turn = (Kp * error) + (Ki*sumError) + (Kd*dError);
//
//        robot.motorLeft.setPower(FinalHardware.SLOW_POWER - turn);
//        robot.motorRight.setPower(FinalHardware.SLOW_POWER + turn);
//
//        loopCounter++;
//        double nextTimeSlot = loopCounter * dt;
//        while (runtime.milliseconds() < nextTimeSlot) {
//            idle();
//        }
//
//
//
//
//    }
//
//    public void calibrateColors(){
//        int alpha = robot.sensorColor.alpha();
//
//        if ((alpha <= minBrightness)) {
//            int minBrightness = alpha; //determine lowest brightness value (grey)
//        }
//        //find the highest values
//        else if (alpha > maxBrightness) {
//            int maxBrightness = alpha; //determine highest brightness value (white)
//        }
//
//        threshold = (maxBrightness + minBrightness)/2;
//    }
}
