package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

@Autonomous(name = "red auto1", group = "red")
public class RedAutonomous1 extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 69;
    public static final float Y_SHOOT = 50;
    public static final float angle1 = -153;
    public static final float angle2 = -160;
    public static final float angle3 = -166;

    GoalBot bot = new GoalBot();

    Rings rings = Rings.ZERO;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        VuforiaNavigator.activate(null, null);
        sleep(3000);
        CameraDevice.getInstance().setField("zoom", ""+20);
        bot.setPose(X0, Y0, 180);
        bot.setKickerUnengaged();
        bot.setGrabberClosed();
        waitForStart();
        bot.setShooterPowerHigh();
        rings = getRings(false);
        driveToPosition(18, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
        turnToHeading(angle1, 1, 6, 45);
        shoot();
        bot.setIntake(GoalBot.IntakeState.FWD);
//        telemetry.addData("shot 1", "");
//        telemetry.update();
        turnToHeading(angle2,1, 6, 45);
        shoot();
//        telemetry.addData("shot 2", "");
//        telemetry.update();
        turnToHeading(angle3,1, 6, 45);
        shoot();
//        telemetry.addData("shot 3", "");
//        telemetry.update();

        turnToHeading(-90, 3, 6, 45);
        float x;
        float y;
        if(rings == Rings.ZERO){
            x = 72;
            y = 24;
        } else if(rings == Rings.ONE){
            x = 96;
            y = 48;
        } else{
            x = 120;
            y = 24;
        }

        //bot.setArmPosition(342);
        driveToPosition(18,4,x,y,-90,2,1);
        bot.setGrabberOpen();
        sleep(500);
        //bot.setArmPosition(120);
        sleep(500);
        float parkY = bot.getPose().y;
        driveToPosition(18,4,84,parkY, -90,2,1);

        while (opModeIsActive()) {
            continue;
        }

    }
}
