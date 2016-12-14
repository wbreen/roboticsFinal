package finalBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.HardwareDriveBot;

import static java.lang.Thread.sleep;

/**
 * @author Beth Lester and William Breen
 * @copyright Beth Lester and William Breen 2016
 *
 * Notes:
 *  Motors are in the front of the robot, sweeper is in the front of the robot
 *      maybe have a button to press that changes forward/backward?
 */

public class FinalHardware {
    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    //Useful constants
    public static final double STOP = 0.0;
    public static final double SLOW_POWER =  0.2;
    public static final double WHEEL_DIAMETER = 3.9;
    public static final double WHEEL_BASE = 18;
    public static final int ENC_ROTATION_40 = 1120;
    public static final int ENC_ROTATION_60 = 1680;
    public static final int SHOULDER_ROTATION = 8690;
    public static final int ELBOW_ROTATION = 5040;

    //------------------------------------sensors ----------------
    public TouchSensor sensorShoulder = null;
    public TouchSensor sensorElbow = null;
    public ColorSensor sensorColor = null;
    public GyroSensor sensorGyro = null;

    //--------------------------------------Motors---------------------------------------
    public DcMotor motorShoulder = null;
    public DcMotor motorElbow = null;
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorSweep = null;

    //limits and settings for the shoulder joint motor (need to experiment and set) (40)
    final static int DELTA_SHOULDER = 25;     //speed of rotation
    final static int INIT_SHOULDER = 0;
    final static double POWER_SHOULDER_SLOW = 0.1;
    final static double POWER_SHOULDER_FAST = .4;

    //limits and settings for the elbow motor (60)
    final static int DELTA_ELBOW = 25;
    final static int INIT_ELBOW = 0;
    final static double POWER_ELBOW_SLOW = .1;
    final static double POWER_ELBOW_FAST = .5;
    //maybe go slow for going around sweeper, then fast for going the full way round

    //Values for turning sweeper on and off
    final static double SWEEPER_ON = 1.0;
    final static double SWEEPER_OFF = 0.0;

    //set initial positions for shoulder and elbow motors
    int posShoulder = INIT_SHOULDER;
    int posElbow = INIT_ELBOW;


    //--------------------------------Servos---------------------------------------------
    Servo servoBucket;
    //limits and settings for the bucket:
    final static double DELTA_BUCKET = 0.01;     //speed of rotation
    final static double MIN_BUCKET = 0.0;
    final static double MAX_BUCKET = 1.0;
    final static double INIT_BUCKET = 0.0;
    double posBucket = INIT_BUCKET;

    Servo servoKickstandRight;
    //limits and settings for the RHS kickstand
    final static double DELTA_KICKSTAND_R = 0.01;
    final static double KICKSTAND_UP_R = .43;
    final static double KICKSTAND_DOWN_R = .81;
    double posKickstandRight = KICKSTAND_UP_R;

    Servo servoKickstandLeft;
    //limits and settings for LHS kickstand
    final static double DELTA_KICKSTAND_L = 0.01;
    final static double KICKSTAND_UP_L = .57;      //TODO: Up position
    final static double KICKSTAND_DOWN_L = .20;   //todo: Down position
    double posKickstandLeft = KICKSTAND_UP_L;

    //Values shared by both Kickstands
    final static double DELTA_KICKSTAND = 0.01;


    //------------------------------------Methods-------------------------------------------

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) throws InterruptedException {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //------------------------------------------------
        // Define and init arm motors
        motorShoulder = hwMap.dcMotor.get("motorShoulder");
        motorShoulder.setDirection(DcMotor.Direction.FORWARD);
        resetShoulderEncoder();

        motorElbow = hwMap.dcMotor.get("motorElbow");
        motorElbow.setDirection(DcMotor.Direction.FORWARD);
        resetElbowEncoder();

        //define and init sweeper motor
        motorSweep = hwMap.dcMotor.get("motorSweep");
        motorSweep.setDirection(DcMotor.Direction.REVERSE);

        // define and init drive motors
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        motorRight = hwMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        resetDriveEncoders();

        //------------------------------------------------
        //Define and init servos:
        servoBucket = hwMap.servo.get("servoBucket");
        servoKickstandRight = hwMap.servo.get("servoKickstandRight");
        servoKickstandLeft = hwMap.servo.get("servoKickstandLeft");

        //------------------------------------------------
        //Define and init sensors:
        sensorShoulder = hwMap.touchSensor.get("sensorShoulder");
        sensorElbow = hwMap.touchSensor.get("sensorElbow");
        sensorGyro = hwMap.gyroSensor.get("sensorGyro");
        sensorColor = hwMap.colorSensor.get("sensorColor");

        sensorColor.enableLed(true);
        sensorGyro.calibrate(); //set 0 heading
        while (sensorGyro.isCalibrating()) {
            Thread.sleep(20);
        }

        //---------------------------------------------
        //init method calls
        initShoulder();
        initServos();
        initElbow();


        finishInit();
    }
    //-----------------Initialization Methods------------------------

    //move the arm up till it hits the sensorShoulder touch sensor, then reset to 0
    public void initShoulder(){
        do{
            int currentShoulderPosition = motorShoulder.getCurrentPosition();
            posShoulder =  currentShoulderPosition + DELTA_SHOULDER;
            motorShoulder.setTargetPosition(posShoulder);
            motorShoulder.setPower(POWER_SHOULDER_SLOW);

            //System.out.println("posShoulder = " + posShoulder);
        } while(!sensorShoulder.isPressed());

        motorShoulder.setPower(STOP);
        resetShoulderEncoder();
        posShoulder = 0;
    }

    //move the shoulder up till it hits the sensorElbow, then reset position to 0
    public void initElbow(){
        do{
            int currentElbowPosition = motorElbow.getCurrentPosition();
            posElbow = currentElbowPosition - DELTA_ELBOW;
            motorElbow.setTargetPosition(posElbow);
            motorElbow.setPower(POWER_ELBOW_SLOW);
        } while(!sensorElbow.isPressed());

        motorElbow.setPower(STOP);
        resetElbowEncoder();
        posElbow = 0;
    }

    //initialize the servos to their starting positions
    public void initServos() throws InterruptedException{
        servoKickstandRight.setPosition(posKickstandRight);
        servoKickstandLeft.setPosition(posKickstandLeft);
        servoBucket.setPosition(posBucket);
        sleep(500);
    }

    //finish the initialization
    public void finishInit(){
        posShoulder = 0;
        motorShoulder.setTargetPosition(posShoulder);
        motorShoulder.setPower(POWER_SHOULDER_SLOW);

        posElbow = 0;
        motorElbow.setTargetPosition(posElbow);
        motorElbow.setPower(POWER_ELBOW_SLOW);

        posBucket = 0.1;
        servoBucket.setPosition(posBucket);

        posKickstandRight = KICKSTAND_UP_R;
        posKickstandLeft = KICKSTAND_UP_L;
        servoKickstandRight.setPosition(posKickstandRight);
        servoKickstandLeft.setPosition(posKickstandLeft);
    }
    //---------------------------Methods used throughout code------------------

    //resetShoulderEncoder --> reset the shoulder encoder to 0
    public void resetShoulderEncoder() {
        motorShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //reset the elbow encoder to 0
    public void resetElbowEncoder() {
        motorElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //reset drive motors to 0
    public void resetDriveEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start(double speed) {
        motorLeft.setPower(-speed); //(motor is on bkwds so set -speed, not pos)
        motorRight.setPower(speed);
    }

    public void spin(double speed) {
        motorLeft.setPower(speed);   // left motor turns backward for CW turn (motor is on bkwds so set +speed, not neg)
        motorRight.setPower(speed);   // right motor turns forward for CW turn
    }

    public void stop() {
        motorLeft.setPower(STOP);
        motorRight.setPower(STOP);
    }

    public void moveRobot (double speed, double inches) throws InterruptedException {
        double direction = Math.signum(speed * inches);

        if(direction == 0.0) return;

        int encoderTarget = convertInchesToTicks(Math.abs(inches));

        // move the desired distance:
        resetDriveEncoders();
        start(Math.abs(speed) * direction);
        while (Math.abs(motorRight.getCurrentPosition()) < encoderTarget) {
            Thread.sleep(20);
        }
        stop();
    }

    public void turnRobot(double speed, double degrees) throws InterruptedException {
        double direction = Math.signum(speed * degrees);
        if(direction == 0.0) return;

        int encoderTarget = convertDegreesToTicks( Math.abs(degrees) );

        resetDriveEncoders();
        spin(Math.abs(speed) * direction);
        while (Math.abs(motorRight.getCurrentPosition()) < encoderTarget) {
            Thread.sleep(20);
        }
        stop();
    }

    public static int convertDegreesToTicks(double degrees) {
        double wheelRotations = (degrees / 360.0) * Math.PI * WHEEL_BASE
                / (Math.PI * WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * ENC_ROTATION_40);

        return encoderTarget;
    }

    public static double convertTicksToInches(int encoderTicks) {
        double wheelRotations = (double) encoderTicks / HardwareDriveBot.ENC_ROTATION;
        double inches = wheelRotations * (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);

        return inches;
    }

    public int convertInchesToTicks(double inches) {
        double wheelRotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTicks = (int)(wheelRotations * HardwareDriveBot.ENC_ROTATION);

        return encoderTicks;
    }

    public void toCollectPos(){ //assuming coming from init/carry pos
        motorShoulder.setTargetPosition(-90);
        motorShoulder.setPower(SLOW_POWER);

        motorElbow.setTargetPosition(141);
        motorElbow.setPower(SLOW_POWER);

        servoBucket.setPosition(0.28);

        motorShoulder.setTargetPosition(-440);
        motorShoulder.setPower(SLOW_POWER);

        motorElbow.setTargetPosition(280);
        motorElbow.setPower(SLOW_POWER);

        motorShoulder.setTargetPosition(-500);
        motorShoulder.setPower(SLOW_POWER);

    }

    public void stopCollect(){
        motorShoulder.setTargetPosition(-387);
        motorShoulder.setPower(SLOW_POWER);

        motorElbow.setTargetPosition(227);
        motorElbow.setPower(SLOW_POWER);

        servoBucket.setPosition(0.222);

        motorSweep.setPower(SWEEPER_OFF);

        motorShoulder.setTargetPosition(0);
        motorShoulder.setPower(SLOW_POWER);
    }

    public void restPos(){
        motorShoulder.setTargetPosition(0);
        motorShoulder.setPower(SLOW_POWER);

        motorElbow.setTargetPosition(0);
        motorElbow.setPower(SLOW_POWER);

        servoBucket.setPosition(0);
    }

    public void bucketOverSweeper()throws InterruptedException{
        motorSweep.setPower(SWEEPER_ON);

        motorElbow.setTargetPosition(650);
        motorElbow.setPower(POWER_ELBOW_SLOW);
        sleep(3000);

        motorSweep.setPower(SWEEPER_OFF);

        servoBucket.setPosition(0.0);

        sleep(2000);

        motorElbow.setTargetPosition(1200);
        motorElbow.setPower(POWER_ELBOW_SLOW);

        posElbow = 1200;
        posBucket = 0.0;
    }

    public void pos30() throws InterruptedException{
        motorShoulder.setTargetPosition(-1992);
        motorShoulder.setPower(POWER_SHOULDER_FAST);


        motorElbow.setTargetPosition(3869);
        motorElbow.setPower(POWER_ELBOW_FAST);

        servoBucket.setPosition(MAX_BUCKET);
        sleep(1000);
        servoBucket.setPosition(posBucket);
    }

    public void pos60() throws InterruptedException{
        motorShoulder.setTargetPosition(-1160);
        motorElbow.setTargetPosition(2271);
        motorElbow.setPower(POWER_ELBOW_SLOW);
        motorShoulder.setPower(POWER_SHOULDER_SLOW);
        sleep(1000);

        servoBucket.setPosition(0.5);
        sleep(1000);

        motorElbow.setTargetPosition(3075);
        motorShoulder.setTargetPosition(-1275);
        motorElbow.setPower(POWER_ELBOW_SLOW);
        motorShoulder.setPower(POWER_SHOULDER_SLOW);
        sleep(1000);

        motorElbow.setTargetPosition(3500);
        motorElbow.setPower(POWER_ELBOW_SLOW);
        sleep(3000);

        servoBucket.setPosition(MAX_BUCKET);
        sleep(1000);

        motorElbow.setTargetPosition(3750);
        motorElbow.setPower(POWER_ELBOW_SLOW);
        sleep(3000);

        servoBucket.setPosition(0.85);
        sleep(500);

        servoBucket.setPosition(0.80);
        sleep(500);

        servoBucket.setPosition(0.70);
        sleep(500);

        servoBucket.setPosition(0.65);
        sleep(500);

        servoBucket.setPosition(0.60);
        sleep(500);

        servoBucket.setPosition(0.55);
        sleep(500);

        servoBucket.setPosition(0.50);
        sleep(500);

        servoBucket.setPosition(0.40);
        sleep(500);

        servoBucket.setPosition(0.30);
        sleep(500);

        motorElbow.setTargetPosition(3000);
        motorElbow.setPower(POWER_ELBOW_FAST);
        sleep(1000);

        posBucket = MAX_BUCKET;
        posElbow = 3000;
        posShoulder = -1275;

    }

    public void returnBucket()throws InterruptedException{
        motorElbow.setTargetPosition(1000);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        motorShoulder.setTargetPosition(0);
        motorShoulder.setPower(SLOW_POWER);
        sleep(1000);

        servoBucket.setPosition(0.78);
        sleep(500);

        motorElbow.setTargetPosition(482);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        servoBucket.setPosition(0.67);
        sleep(500);

        motorElbow.setTargetPosition(316);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        servoBucket.setPosition(0.43);
        sleep(500);

        motorElbow.setTargetPosition(225);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        servoBucket.setPosition(0.33);
        sleep(500);

        motorElbow.setTargetPosition(154);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        servoBucket.setPosition(0.21);
        sleep(500);

        motorElbow.setTargetPosition(71);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        motorSweep.setPower(SWEEPER_ON);
        sleep(500);

        servoBucket.setPosition(0);
        sleep(500);

        motorElbow.setTargetPosition(0);
        motorElbow.setPower(SLOW_POWER);
        sleep(1000);

        posBucket = 0;
        posElbow = 0;
        posShoulder = 0;

        sleep(1000);

    }

    public void kickstandDown(){
        servoKickstandLeft.setPosition(KICKSTAND_DOWN_L);
        servoKickstandRight.setPosition(KICKSTAND_DOWN_R);
    }

    public void kickstandUp(){
        servoKickstandLeft.setPosition(KICKSTAND_UP_L);
        servoKickstandRight.setPosition(KICKSTAND_UP_R);
    }

    //implements periodic delay ("metronome")
    public void waitForTick(long periodMs) throws InterruptedException{
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
