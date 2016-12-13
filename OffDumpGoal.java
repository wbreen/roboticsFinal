

package finalBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Beth Lester on 12/1/2016.
 */

@Autonomous(name="DumpGoal", group="ElonDev")
//@Disabled

public class OffDumpGoal extends LinearOpMode {
    FinalHardware robot = new FinalHardware();

    public boolean isRedTeam;
    public boolean isBlueTeam;

    private double Pc = 0.5; //oscillation period
    private double Kc = 0.01; //critical gain
    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in seconds
    private double Kp = (0.6*Kc)/2; //0.6*Kc is giving about 2x our ideal Kp
    private double Ki = (2*Kp*dT)/Pc;
    private double Kd = (Kp*Pc)/(8*dT);

    private int reference = 27; //determined using basic min/max average of brightnesses
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
        int red = robot.sensorColor.red();
        int blue = robot.sensorColor.blue();

        if (red > blue){
            isRedTeam = true;
            isBlueTeam = false;
        }
        else if (blue > red){
            isRedTeam = false;
            isBlueTeam = true;
        }

        sleep(3000);

        //get off ramp
        robot.moveRobot(robot.SLOW_POWER,40);
        //sleep at end of ramp for testing
        sleep(5000);


        while (opModeIsActive()){
            int currentHeading = robot.sensorGyro.getHeading();
            if((currentHeading >= 180) && (currentHeading <= 359)){
                robot.turnRobot(robot.SLOW_POWER,360-currentHeading);
            }
            else if((currentHeading >= 180) && (currentHeading <= 359)){
                robot.turnRobot(-robot.SLOW_POWER,currentHeading);
            }
            do{
                inches = robot.convertTicksToInches(robot.motorRight.getCurrentPosition());

                robot.resetDriveEncoders();

                double brightness = robot.sensorColor.alpha();
                error = reference - brightness;

                sumError = 0.9*sumError + error;
                dError = error - prevError;
                prevError = error;

                double turn = (Kp * error) + (Ki*sumError) + (Kd*dError);

                robot.motorLeft.setPower(-robot.SLOW_POWER - turn);
                robot.motorRight.setPower(-robot.SLOW_POWER + turn);

                loopCounter++;
                double nextTimeSlot = loopCounter * dt;
                while (runtime.milliseconds() < nextTimeSlot) {
                    idle();
                }
            }while(inches <= 30);

            robot.stop();
            robot.kickstandDown();
            robot.moveRobot(robot.SLOW_POWER,2); //just to be sure it's in the right spot
            robot.pos60();

            robot.moveRobot(-robot.SLOW_POWER,30);
            robot.turnRobot(robot.SLOW_POWER,-90);
            robot.moveRobot(robot.SLOW_POWER, 20);

        }
    }
}
