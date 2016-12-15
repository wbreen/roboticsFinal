
package finalBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Beth Lester and William Breen
 * @copyright Beth Lester and William Breen
 */

@TeleOp(name="Final Manual", group="Final Robot")  // @Autonomous(...) is the other common choice

public class FinalManual extends LinearOpMode{
    FinalHardware robot = new FinalHardware();
    private ElapsedTime runtime = new ElapsedTime();

    //boolean startButtonPressed = false;

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
            int currentHeading = robot.sensorGyro.getHeading();

            //--------------------------------move shoulder up and down------------------
            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();
            if (gamepad1.left_trigger > 0.5){
                //move the shoulder down
                robot.posShoulder = currentShoulderPosition - robot.DELTA_SHOULDER;
            }
            if (gamepad1.left_bumper){
                //move shoulder up
                if (robot.sensorShoulder.isPressed()){
                    robot.resetShoulderEncoder();
                    robot.posShoulder = 0;
                } else {
                    robot.posShoulder = currentShoulderPosition + robot.DELTA_SHOULDER;
                }
            }
            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(robot.POWER_SHOULDER_SLOW);

            //----------------------------------move elbow up and down-----------------------------
            int currentElbowPosition = robot.motorElbow.getCurrentPosition();
            if (gamepad1.right_trigger > 0.5){
                //move elbow down
                if(robot.sensorElbow.isPressed()) {
                    robot.resetElbowEncoder();
                    robot.posElbow = 0;
                } else {
                    robot.posElbow = currentElbowPosition - robot.DELTA_ELBOW;
                }
            }
            if (gamepad1.right_bumper){
                //move elbow up
                robot.posElbow = currentElbowPosition + robot.DELTA_ELBOW;
            }
            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW_SLOW);
            int elbowTicks = robot.motorElbow.getCurrentPosition();


            //-----------------------------------drive robot-----------------------------------------

            double speedRight = .5*(-gamepad1.right_stick_y);
            double speedLeft = .5*(gamepad1.left_stick_y);

            robot.motorLeft.setPower(speedLeft);
            robot.motorRight.setPower(speedRight);
            //possibly change this to a toggle if time
            int driveTicks = robot.motorLeft.getCurrentPosition();

            //---------------------------------Sweeper on - off----------------------------
            if(gamepad1.a){
                robot.motorSweep.setPower(robot.SWEEPER_ON);
            }
            if (gamepad1.b){
//                robot.motorSweep.setPower(robot.SWEEPER_OFF);
                robot.bucketOverSweeper();
            }

            //-------------------------------Kickstand Movement---------------------
            if (gamepad1.dpad_up){
                robot.kickstandUp();
            }
            if (gamepad1.dpad_down) {
                robot.kickstandDown();
            }

            //-------------------------------Bucket Movement---------------------
            //move the bucket forward
            if (gamepad1.x){
                robot.posBucket = Range.clip(robot.posBucket + robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }
            //move the bucket back
            if (gamepad1.y){
                robot.posBucket = Range.clip(robot.posBucket - robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }
            robot.servoBucket.setPosition(robot.posBucket);

            //TODO: LOOK AT AND EVENTUALLY REMOVE TELEMETRY STUFF
//            telemetry.addData("Shoulder Motor Pos", robot.posShoulder);
//            telemetry.addData("Elbow Motor Pos", robot.posElbow);
//            telemetry.addData("Pos Bucket", robot.posBucket);
//            telemetry.addData("heading", currentHeading);
//            telemetry.addData("Color", String.format("R=%3d G=%3d B=%3d A=%3d", robot.sensorColor.red(),
//                    robot.sensorColor.green(), robot.sensorColor.blue(), robot.sensorColor.alpha()));
//            telemetry.update();

            //-------------------------------------------Automated movements-----------------------------------------------
            if(gamepad2.dpad_up){
                robot.bucketOverSweeper();
            }
            if(gamepad2.dpad_right){
                robot.stopCollect();
            }
            if(gamepad2.dpad_left){
                robot.restPos();
            }
            if(gamepad2.dpad_down){
                robot.returnBucket();
            }
            if(gamepad2.y){
                robot.pos90();
            }
            if(gamepad2.a){
                robot.pos60();
            }


            //-------------------------------------------wrapping up-----------------------------------------------------
            robot.waitForTick(20);      //time waiting at end of loop in MS

            idle();
        }
    }

}