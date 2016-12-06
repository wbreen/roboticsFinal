package finalBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Beth Lester and William Breen
 * @copyright Beth Lester and William Breen
 */

@TeleOp(name="FinalManula", group="Elon")  // @Autonomous(...) is the other common choice

public class FinalManual extends LinearOpMode{
    FinalHardware robot = new FinalHardware();
    private ElapsedTime runtime = new ElapsedTime();

    boolean startButtonPressed = false;

    @Override
    public void runOpMode() throws  InterruptedException{
        //initialize the robot
        robot.init(hardwareMap);
        telemetry.addData("Status","Finished Init");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //runs once "play" is pressed but stops once "stop" is pressed
        while(opModeIsActive()) {
            // reset shoulder and elbow encoders by pressing the Start button
            if (gamepad1.start && !startButtonPressed){
                //start button state went from not pressed to pressed
                startButtonPressed = true;
                robot.resetShoulderEncoder();
                robot.resetElbowEncoder();
                robot.posElbow = 0;
                robot.posShoulder = 0;
            }
            else{
                startButtonPressed = false;
            }

            //--------------------------------move shoulder up and down------------------
            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();
            if (gamepad1.left_trigger > 0.5){
                //move the shoulder down
                robot.posShoulder = currentShoulderPosition + robot.DELTA_SHOULDER;
            }
            if (gamepad1.left_bumper){
                //move shoulder up
                if (robot.sensorArm.isPressed()){
                    robot.resetShoulderEncoder();
                    robot.posShoulder = 0;
                } else {
                    robot.posShoulder = currentShoulderPosition - robot.DELTA_SHOULDER;
                }
            }
            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(robot.POWER_SHOULDER);
            int shoulderTicks = robot.motorShoulder.getCurrentPosition();


            //----------------------------------move elbow up and down-----------------------------
            int currentElbowPosition = robot.motorElbow.getCurrentPosition();
            if (gamepad1.right_trigger > 0.5){
                //move elbow down
                robot.posElbow = currentElbowPosition + robot.DELTA_ELBOW;
            }
            if (gamepad1.right_bumper){
                //move elbow up
                if(robot.sensorElbow.isPressed()) {
                    robot.resetElbowEncoder();
                    robot.posElbow = 0;
                } else{
                    robot.posElbow = currentElbowPosition - robot.DELTA_ELBOW;
                }

            }
            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW);
            int elbowTicks = robot.motorElbow.getCurrentPosition();


            //-----------------------------------drive robot-----------------------------------------
            double speed = gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;

            double speedLeft = Range.clip(speed + turn, -1.0, 1.0);
            double speedRight = Range.clip(speed - turn, -1.0, 1.0);

            robot.motorLeft.setPower(speedLeft);
            robot.motorRight.setPower(speedRight);
            int driveTicks = robot.motorLeft.getCurrentPosition();


            //-------------------------------Kickstand Movement---------------------
            if (gamepad1.dpad_up){
                robot.kickstandUp();
            }
            if (gamepad1.dpad_down) {
                robot.kickstandDown();
            }


            //-------------------------------Bucket Movement---------------------
            //move the bucket forward until the lowest pos is reached
            if (gamepad1.x){
                robot.posBucket = Range.clip(robot.posBucket + robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }
            //move the bucket back (dump pos)
            if (gamepad1.right_stick_x < -0.3){
                robot.posBucket = Range.clip(robot.posBucket - robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }


            robot.waitForTick(50);      //time waiting at end of loop in MS
        }
    }

}
