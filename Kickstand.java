package finalBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * @author Beth Lester and William Breen
 * @copyright Beth Lester and William Breen
 */

@Autonomous(name="Kickstand", group="ElonDev")

public class Kickstand extends LinearOpMode{
    FinalHardware robot = new FinalHardware();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
    }
}
