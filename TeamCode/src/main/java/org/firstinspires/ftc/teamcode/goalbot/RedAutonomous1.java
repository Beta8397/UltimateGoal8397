package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

@Autonomous(name = "red auto1", group = "red")
public class RedAutonomous1 extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 66;
    public static final float Y_SHOOT = 59;

    GoalBot bot = new GoalBot();

    Rings rings = Rings.ZERO;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        VuforiaNavigator.activate(null, null);
        bot.setPose(X0, Y0, 180);
        waitForStart();
        rings = getRings();
        driveToPosition(12, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
    }
}
