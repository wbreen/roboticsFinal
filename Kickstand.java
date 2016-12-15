package finalBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * @author Beth Lester and William Breen
 * @copyright Beth Lester and William Breen 2016
 *
 */

@Autonomous(name="Kickstand", group="ElonDev")

public class Kickstand extends LinearOpMode{
    FinalHardware robot = new FinalHardware();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.moveRobot(robot.SLOW_POWER, 20);
        robot.turnRobot(robot.SLOW_POWER, -90);
        robot.moveRobot(robot.SLOW_POWER, 30);
        robot.turnRobot(robot.SLOW_POWER, 90);
        robot.moveRobot(robot.SLOW_POWER, 35);
        robot.turnRobot(robot.SLOW_POWER, 90);
        robot.moveRobot(0.5,30);
    }
}
