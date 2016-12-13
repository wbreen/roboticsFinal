

package finalBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Beth Lester on 12/1/2016.
 */

@Autonomous(name="DumpGoal", group="ElonDev")

public class OffDumpGoal extends LinearOpMode {
    FinalHardware robot = new FinalHardware();

    private boolean isRedTeam;
    private boolean isBlueTeam;
    private int red;
    private int blue;
    private int alpha;


    private double Pc = 0.8; //oscillation period
    private double Kc = 0.005; //critical gain
    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in seconds
    private double Kp = (0.6*Kc);
    private double Ki = (2*Kp*dT)/Pc;
    private double Kd = (Kp*Pc)/(8*dT);

    // //66 9 64 10 61 8 ALPHA
// //23 6 20 6 20 6 BLUE
// 26 7 27 7 33 7 36 8 30 RED

    private int reference = 37; //determined using basic min/max average of brightnesses
    private double error = 0.0;
    private double sumError = 0.0;
    private double dError = 0.0;
    private double prevError = 0.0;

    private int loopCounter = 0;
    private double inches;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode()throws InterruptedException {
        //initialize the robot
        robot.init(hardwareMap);

        sleep(1000);

        //move arm to driving pos;
        robot.carryPos();

        sleep(3000);

        //determine which color we're looking for
        red = robot.sensorColor.red();
        blue = robot.sensorColor.blue();

        if (red > blue){
            isRedTeam = true;
            telemetry.addData("red",red);
            telemetry.update();
            isBlueTeam = false;
        }
        else {
            isRedTeam = false;
            telemetry.addData("blue",blue);
            telemetry.update();
            isBlueTeam = true;
        }

        sleep(2000);

        //get off ramp
        robot.moveRobot(robot.SLOW_POWER,40);
        //sleep at end of ramp for testing
        sleep(2000);

        int currentHeading = robot.sensorGyro.getHeading();
        telemetry.addData("heading", currentHeading);
        telemetry.update();

        //find the white line
        if((currentHeading > 180) && (currentHeading < 359)){
            do{
                alpha  = robot.sensorColor.alpha();
                robot.turnRobot(robot.SLOW_POWER,1);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("alpha",alpha);
                telemetry.update();
            }while(alpha < reference);
        }
        else if((currentHeading < 180) && (currentHeading > 1)){
            do{
                alpha  = robot.sensorColor.alpha();
                robot.turnRobot(robot.SLOW_POWER,-1);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("alpha",alpha);
                telemetry.update();
            }while(alpha < reference);
        }

        //get on the edge of the line
        do{
            alpha  = robot.sensorColor.alpha();
            error = reference - alpha;
            robot.turnRobot(-robot.SLOW_POWER,1);
            telemetry.addData("alpha", alpha);
            telemetry.update();
        } while(error >= 0.05);

        sleep(2000);

        robot.resetDriveEncoders();
        error = 0.0;

        runtime.reset();

        //follow white line for 20 in
        do{
            inches = robot.convertTicksToInches(robot.motorRight.getCurrentPosition());
            alpha = robot.sensorColor.alpha();
            error = reference - alpha;

            sumError = 0.9*sumError + error;
            dError = error - prevError;
            prevError = error;

            double turn = (Kp * error) + (Ki*sumError) + (Kd*dError);

            robot.motorLeft.setPower(-0.2 - turn);
            robot.motorRight.setPower(0.2 + turn);

            loopCounter++;
            double nextTimeSlot = loopCounter * dt;
            while (runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }while(inches < 30);

        //grab the goal + dump
        robot.stop();
        robot.kickstandDown();
        robot.moveRobot(robot.SLOW_POWER,2); //just to be sure it's in the right spot
        robot.bucketOverSweeper();
        robot.pos60();
        robot.returnBucket();


        //go bring goal to rectangle
        robot.moveRobot(-robot.SLOW_POWER,30);
        robot.turnRobot(robot.SLOW_POWER,-90);
        robot.moveRobot(robot.SLOW_POWER, 20);
        robot.turnRobot(robot.SLOW_POWER,-90);

        if(isRedTeam){
            do{
                robot.moveRobot(robot.SLOW_POWER, 2);
            }while((robot.sensorColor.red() > red + 5) || (robot.sensorColor.red() < red - 5));
        }

        else {
            do{
                robot.moveRobot(robot.SLOW_POWER, 2);
            }while((robot.sensorColor.blue() > blue + 5) || (robot.sensorColor.blue() < blue - 5));
        }
    }
}
