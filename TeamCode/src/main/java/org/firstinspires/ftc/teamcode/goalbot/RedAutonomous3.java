package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

import org.apache.commons.math3.special.Beta;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous(name = "red auto3", group = "red")
public class RedAutonomous3 extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 36f;
    public static final float Y_SHOOT = 34;
    public static final float shootingAngle = -164;
//    public static final float angle1 = -160;
//    public static final float angle2 = -164;
//    public static final float angle3 = -170;

    GoalBot bot = new GoalBot();

    Rings rings = Rings.ZERO;

    public void runLoggingOpMode() {
        imuInitialized = bot.init(hardwareMap);
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
        testGyroAndWaitForStart();
        bot.setShooterPower(0.72f);
        rings = getRings(true);
        driveToPosition(18, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
        turnToHeading(shootingAngle, 1, 6, 45);
//        shoot1();
//        //bot.setIntake(GoalBot.IntakeState.FWD);
////        telemetry.addData("shot 1", "");
////        telemetry.update();
////        turnToHeading(angle2,1, 6, 45);
//        sleep(150);
//        bot.setRingKickerEngaged();
//        sleep(150);
//        shoot2();
////        telemetry.addData("shot 2", "");
////        telemetry.update();
////        turnToHeading(angle3,1, 6, 45);
//        sleep(300);
//        shoot2();
//        bot.setRingKickerUnengaged();
////        telemetry.addData("shot 3", "");
////        telemetry.update();
        BetaLog.d("shooter speed", bot.shooter.getVelocity());

        bot.setKickerEngaged();
        sleep(500);
        bot.setShooterPower(0.68f);
        bot.setKickerUnengaged();
        //bot.setShooterPower(0.7f);
        sleep(500);
        BetaLog.d("shooter speed", bot.shooter.getVelocity());

        bot.setKickerEngaged();
        bot.setRingKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
        //bot.setRingKickerEngaged();
        sleep(500);
        BetaLog.d("shooter speed", bot.shooter.getVelocity());

        bot.setKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
        bot.setRingKickerUnengaged();

        bot.setIntake(GoalBot.IntakeState.FWD);

        turnToHeading(-90, 5, 8, 60);
        if (rings == Rings.ONE) {
            driveToPosition(36,6, bot.getPose().x,53,-90,2,1);
        } else {
            driveToPosition(36,6, bot.getPose().x + 7,24,-90,2,1);
        }
        float y;
        float x;
        if(rings == Rings.ZERO){
            x = 74;
            y = 27;
        } else if(rings == Rings.ONE){
            x = 94;
            y = 52;
        } else{
            x = 123;
            y = 27;
        }

        //Drive to drop off first wobble.

        bot.setArmPosition(470);
        if(rings == Rings.ONE) {
            driveToPosition(new MotionProfile(8, 48, 25), x, y, -90, 1);
        } else {
            driveToPosition(36,6,x,y,-90,2,1);
        }
        //driveToPosition(36,6,x,y,-90,2,1);
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
            driveToPosition(36, 6, 34, 36, 180, 2, 1);
            turnToHeading(-170, 3,8,60);
        } else if (rings == Rings.ONE){
//            driveToPosition(36, 6, bot.getPose().x - 24, 32, 180, 2, 6 );
//            driveToPosition(36, 6, 36, 32, 180, 2, 1);
            driveToPosition(new MotionProfile(8, 48, 25), bot.getPose().x - 24, 36, 180, 6 );
            driveToPosition(new MotionProfile(8, 48, 25), 36, 35, 180, 1);
            turnToHeading(-170, 3,8,60);
        } else {
            bot.setIntake(GoalBot.IntakeState.REV);
            driveToPosition(36, 6, 16, 44, -90, 2, 1);
            driveToPosition(36, 6, 19, 44, -90, 2, 1);
            driveToPosition(36, 6, 19, 35.5f, -90, 2, 1);
        }
        bot.setGrabberClosed();
        sleep(500);
        bot.setArmPosition(300);
        //Turn robot, then drive to drop off second wobble.
        if (rings != Rings.FOUR) {
            turnToHeading(-90, 6,8, 60);
//            turnToHeading(-90, 6, 8, 60,
//                    new Action() {
//                        @Override
//                        public void update() {
//                            float pos = bot.getArmActualPos();
//                        }
//                    });
        }
        if (rings == Rings.ONE) {
            x = x - 9;
            y = y + 9;
        } else if (rings == Rings.ZERO) {
            x = x - 7;
            y = y + 9;
        } else {
            x = x - 7;
            y = y + 7;
        }

        if(rings == Rings.ONE) {
            driveToPosition(new MotionProfile(8, 48, 25), x, y, -90, 1);
        } else {
            driveToPosition(36, 6, x, y, -90, 2, 1);
        }
        //driveToPosition(36, 6, x, y, -90, 2, 1);
        bot.setArmPosition(470);
        sleep(400);
        bot.setGrabberOpen();
        sleep(300);
        bot.setArmPosition(0);
        bot.setGrabberClosed();
        float parkY = bot.getPose().y;
        float parkX;
        if (rings == Rings.ONE) {
            parkX = 78;
        } else if (rings == Rings.FOUR) {
            parkX = 78;
        } else {
            parkX = 78;
        }
        //drive to park.
        if (rings == Rings.ONE || rings == Rings.FOUR) {
            driveToPosition(36,6, parkX,parkY, -90,2,1);
        } else {
            driveToPosition(36,6, parkX, bot.getPose().y + 3, -90,2,1);
        }

        bot.setShooterPower(0);

    }
}
