

package finalBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Beth Lester on 12/1/2016.
 */

@Autonomous(name="RampDump", group="ElonDev")

public class RampDump extends LinearOpMode {
    FinalHardware robot = new FinalHardware();

    private int alpha;

    private double Pc = 0.7; //oscillation period
    private double Kc = 0.005; //critical gain
    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in seconds
    private double Kp = (0.6*Kc);
    private double Ki = (2*Kp*dT)/Pc;
    private double Kd = (Kp*Pc)/(8*dT);

    private double reference = 25;

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

        telemetry.addData("FinishedInit", "done");
        telemetry.update();

        waitForStart();

        //get off ramp
        robot.moveRobot(0.3,40);
        //sleep at end of ramp for testing
        sleep(500);

        int currentHeading = robot.sensorGyro.getHeading();

        //find the white line
        if((currentHeading > 180) && (currentHeading < 359)){
            do{
                alpha  = robot.sensorColor.alpha();
                robot.turnRobot(0.2,1);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("alpha",alpha);
                telemetry.update();
            }while(alpha < reference);

            do{
                alpha  = robot.sensorColor.alpha();
                error = reference - alpha;
                robot.turnRobot(0.2,-1);
                telemetry.addData("alpha", alpha);
                telemetry.update();
            } while(error > 0.05);
        }
        else if((currentHeading < 180) && (currentHeading > 1)){
            do{
                alpha  = robot.sensorColor.alpha();
                robot.turnRobot(0.2,-1);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("alpha",alpha);
                telemetry.update();
            }while(alpha < reference);

            do{
                alpha  = robot.sensorColor.alpha();
                error = reference - alpha;
                robot.turnRobot(0.2,1);
                telemetry.addData("alpha", alpha);
                telemetry.update();
            } while(error > 0.05);
        }
        sleep(500);

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

            robot.motorLeft.setPower(-0.5 - turn);
            robot.motorRight.setPower(0.5 + turn);

            loopCounter++;
            double nextTimeSlot = loopCounter * dt;

            telemetry.addData("alpha",alpha);
            telemetry.update();

            while (runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }while(inches < 30.0);

        //grab the goal + dump
//        robot.stop();
//        robot.kickstandDown();
//        robot.moveRobot(robot.SLOW_POWER,2); //just to be sure it's in the right spot
//        robot.bucketOverSweeper();
//        sleep(1000);
//        robot.pos60();
//        sleep(3000);
//        robot.returnBucket();

    }
}
