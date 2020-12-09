package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

@Autonomous(name = "red auto2", group = "red")
public class RedAutonomous2 extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 67.5f;
    public static final float Y_SHOOT = 59;
    public static final float angle1 = -160;
    public static final float angle2 = -164;
    public static final float angle3 = -170;

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
        bot.setRingKickerUnengaged();
        bot.setGrabberClosed();
        bot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.setArmPosition(50);
        waitForStart();
        bot.setShooterPowerNormal();
        rings = getRings(true);
        driveToPosition(18, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
        turnToHeading(angle1, 1, 6, 45);
        shoot();
        bot.setIntake(GoalBot.IntakeState.FWD);
//        telemetry.addData("shot 1", "");
//        telemetry.update();
        turnToHeading(angle2,1, 6, 45);
        bot.setRingKickerEngaged();
        shoot();
        bot.setRingKickerUnengaged();
//        telemetry.addData("shot 2", "");
//        telemetry.update();
        turnToHeading(angle3,1, 6, 45);
        shoot();
//        telemetry.addData("shot 3", "");
//        telemetry.update();

        turnToHeading(-90, 5, 8, 60);
        float x;
        float y;
        if(rings == Rings.ZERO){
            x = 75;
            y = 27;
        } else if(rings == Rings.ONE){
            x = 97;
            y = 51;
        } else{
            x = 123;
            y = 27;
        }

        //Drive to drop off first wobble.

        bot.setArmPosition(470);
        driveToPosition(36,6,x,y,-90,2,1);
        bot.setGrabberOpen();
        sleep(300);
        bot.setArmPosition(0);
        sleep(300);
        //Drive to pick up the second wobble. Turn first if needed.
        if (rings != Rings.FOUR) {
            turnToHeading(180, 5, 8, 60);
        }
        bot.setArmPosition(610);
        //drive to pick up second wobble.
        if (rings == Rings.ZERO) {
            driveToPosition(36, 6, 36, 29, 180, 2, 1);
        } else if (rings == Rings.ONE){
            driveToPosition(36, 6, bot.getPose().x - 24, 29, 180, 2, 6 );
            driveToPosition(36, 6, 36, 29, 180, 2, 1);
        } else {
            driveToPosition(36, 6, 16, 42, -90, 2, 1);
            driveToPosition(36, 6, 19, 42, -90, 2, 1);
            driveToPosition(36, 6, 19, 36, -90, 2, 1);
        }
        bot.setGrabberClosed();
        sleep(500);
        bot.setArmPosition(300);
        //Turn robot, then drive to drop off second wobble.
        if (rings != Rings.FOUR) {
            turnToHeading(-90, 4,8, 60);
        }
        if (rings != Rings.ZERO) {
            x = x - 7;
        }
        driveToPosition(36,6,x,y,-90,2,1);
        bot.setArmPosition(470);
        sleep(300);
        bot.setGrabberOpen();
        sleep(300);
        bot.setArmPosition(0);
        bot.setGrabberClosed();
        float parkY = bot.getPose().y;
        float parkX = 80;
        //drive to park.
        if (rings == Rings.ONE) {
            driveToPosition(36,6, parkX,parkY, -90,2,1);
        } else {
            driveToPosition(36,6,bot.getPose().x, bot.getPose().y + 3, -90,2,1);
        }

        bot.setShooterPower(0);

    }
}
