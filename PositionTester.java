package finalBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by William Breen on 12/14/2016.
 */

@TeleOp(name="Position Tester", group="Final Robot")


public class PositionTester extends LinearOpMode {
    FinalHardware robot = new FinalHardware();
    private ElapsedTime runtime = new ElapsedTime();

    boolean kickstandDown = false;

    @Override
    public void runOpMode() throws InterruptedException{
        //init robot
        robot.init(hardwareMap);

        telemetry.addData("Status", "Finished Init");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

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
            telemetry.addData("Shoulder Motor Pos", robot.posShoulder);
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
            telemetry.addData("Elbow Motor Pos", robot.posElbow);
            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW_SLOW);
            int elbowTicks = robot.motorElbow.getCurrentPosition();

            if (gamepad1.x) {
                robot.posBucket = Range.clip(robot.posBucket + robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }
            //move the bucket back
            if (gamepad1.y){
                robot.posBucket = Range.clip(robot.posBucket - robot.DELTA_BUCKET, robot.MIN_BUCKET, robot.MAX_BUCKET);
            }
            robot.servoBucket.setPosition(robot.posBucket);
            telemetry.addData("Bucket Position", robot.posBucket);

            if (gamepad1.a){
                robot.pos90();
            }
            if (gamepad1.b){
                if(!kickstandDown){
                    robot.kickstandDown();
                    kickstandDown = true;
                }else {
                    robot.kickstandUp();
                    kickstandDown = false;
                }

            }
            //update telemetry
            telemetry.update();

            robot.waitForTick(50);

            idle();
        }
    }
}
