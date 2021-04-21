package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous(name = "red auto3 extra both", group = "red")
public class RedAutonomous3ExtraBoth extends GoalBotAutonomous {

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
        bot.setGateUp();
        bot.setRingKickerUnengaged();
        bot.setGrabberClosed();
        bot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.setArmPosition(50);
        testGyroAndWaitForStart();
        if(rings != Rings.FOUR){
            bot.setShooterPower(0.74f);
        } else{
            bot.setShooterPower(0.76f);
        }

        ElapsedTime ringTimer = new ElapsedTime();
        rings = getRings(false);
        telemetry.addData("Ring time ", ringTimer.milliseconds());
        telemetry.update();
        if(rings == Rings.FOUR) {
            driveToPosition(36, 8, X_SHOOT, Y_SHOOT, 180, 2, 1);
            turnToHeading(shootingAngle, 1, 8, 60);
        }else {
            driveToPosition(18, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
            turnToHeading(shootingAngle, 1, 6, 45);
        }

//        BetaLog.d("shooter speed", bot.shooter.getVelocity());

        bot.setIntake(GoalBot.IntakeState.FWD);
        bot.setKickerEngaged();
        sleep(500);
        bot.setShooterPower(0.72f);
        bot.setKickerUnengaged();
        sleep(500);
//        BetaLog.d("shooter speed", bot.shooter.getVelocity());


        bot.setKickerEngaged();
        //bot.setRingKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
        //bot.setRingKickerEngaged();
        sleep(500);
//        BetaLog.d("shooter speed", bot.shooter.getVelocity());

        //sleep(500);
        bot.setKickerEngaged();
        sleep(500);

        bot.setIntake(GoalBot.IntakeState.OFF);

        bot.setKickerUnengaged();
        if(rings == Rings.ONE) {
            bot.setShooterPower(0.73f);
        } else bot.setShooterPower(0.74f);
        //bot.setRingKickerUnengaged();

        bot.setIntake(GoalBot.IntakeState.FWD);
        if (rings != Rings.FOUR) {
            turnToHeading(-90, 5, 8, 60);
        }
        if (rings == Rings.ONE) {
            driveToPosition(36,6, bot.getPose().x,53,-90,2,1);
        } else if(rings == Rings.ZERO){
            driveToPosition(36,6, bot.getPose().x + 7,24,-90,2,1);
        } else {
            driveToPosition(36,6, bot.getPose().x + 6,22, shootingAngle + 10,2,1);
            turnToHeading(-90, 5, 8, 60);
        }


        float y;
        float x;
        if(rings == Rings.ZERO){
            x = 74;
            y = 26;
        } else if(rings == Rings.ONE){
            x = 93;
            y = 49;
        } else{
            x = 114;
            y = 27;
        }

        //Drive to drop off first wobble.

        bot.setGateDown();
        bot.setArmPosition(470);
        if(rings == Rings.ONE) {
            driveToPosition(new MotionProfile(8, 48, 25), x, y, -90, 1);
        } else if (rings == Rings.ZERO) {
            driveToPosition(36,6,x,y,-90,2,1);
        } else {
            driveToPosition(new MotionProfile(12, 72, 36), x, y, -90, 1);
        }
        //driveToPosition(36,6,x,y,-90,2,1);
        bot.setGrabberOpen();
        sleep(300);
        bot.setArmPosition(0);
        sleep(300);
        //Drive to pick up the second wobble. Turn first if needed.
        if (rings != Rings.FOUR) {
            turnToHeading(180, 5, 8, 60);
            bot.setArmPosition(590);
        }


        //drive to pick up second wobble.
        if (rings == Rings.ZERO) {
            driveToPosition(36, 6, 35, 33.5f, 180, 2, 1);
            turnToHeading(-170, 3,8,60);
        } else if (rings == Rings.ONE){
//            driveToPosition(36, 6, bot.getPose().x - 24, 32, 180, 2, 6 );
//            driveToPosition(36, 6, 36, 32, 180, 2, 1);
            driveToPosition(new MotionProfile(8, 48, 25), bot.getPose().x - 24, 36, 180, 6 );
            driveToPosition(new MotionProfile(8, 48, 25), 33, 33, 180, 1);
            turnToHeading(-170, 3,8,60);
        } else {
            driveToPosition(new MotionProfile(12, 72, 36), bot.getPose().x - 24, 36, -90, 6 );
            bot.setArmPosition(590);
            turnToHeading(180, 5, 8, 60);
            driveToPosition(new MotionProfile(12, 72, 36),60, 36, 180,2);
            driveToPosition(new MotionProfile(8, 16, 25), X_SHOOT + 5, Y_SHOOT + 3, shootingAngle - 2, 1);

            sleep(250);
            bot.setKickerEngaged();
            sleep(500);
            bot.setShooterPower(0.72f);
            bot.setKickerUnengaged();
            /*sleep(500);


            bot.setKickerEngaged();
            sleep(500);
            bot.setKickerUnengaged();*/


            driveToPosition(new MotionProfile(8, 48, 25), 33, 36, 180, 1);
            turnToHeading(-170, 3,8,60);
        }
        bot.setGrabberClosed();
        sleep(500);
        bot.setArmPosition(300);
        //Turn robot, then drive to drop off second wobble.
        if (rings == Rings.ZERO) {
            turnToHeading(-90, 6,8, 60);
//            turnToHeading(-90, 6, 8, 60,
//                    new Action() {
//                        @Override
//                        public void update() {
//                            float pos = bot.getArmActualPos();
//                        }
//                    });
        } else if(rings == Rings.ONE){
            driveToPosition(new MotionProfile(8, 48,25), X_SHOOT, Y_SHOOT, shootingAngle, 1f);
            turnToHeading(shootingAngle, 2f, 8, 60);

            bot.setKickerEngaged();
            sleep(500);
            bot.setShooterPower(0.7f);
            bot.setKickerUnengaged();
            BetaLog.d("shooter speed", bot.shooter.getVelocity());

            turnToHeading(-90, 5f, 8, 60);
        } else {
//            driveToPosition(new MotionProfile(8, 48,25), X_SHOOT, Y_SHOOT, shootingAngle, 1f);
//            turnToHeading(shootingAngle, 2f, 8, 60);
//
//            bot.setKickerEngaged();
//            sleep(500);
//            bot.setShooterPower(0.7f);
//            bot.setKickerUnengaged();
//            sleep(500);
//
//
//            bot.setKickerEngaged();
//            sleep(500);
//            bot.setKickerUnengaged();

            turnToHeading(-90, 5f, 8, 60);
        }
        bot.setGateUp();
        if (rings == Rings.ONE) {
            x = x - 9;
            y = y + 7;
        } else if (rings == Rings.ZERO) {
            x = x - 7;
            y = y + 7;
        } else {
            x = x - 11;
            y = y + 5;
        }

        if(rings == Rings.ONE) {
            driveToPosition(new MotionProfile(8, 48, 25), x, y, -90, 1);
        } else if (rings == Rings.ZERO){
            driveToPosition(36, 6, x, y, -90, 2, 1);
        } else {
            driveToPosition(new MotionProfile(12, 72, 36), x, y, -90, 1);
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
            parkX = 76;
        } else if (rings == Rings.FOUR) {
            parkX = 84;
        } else {
            parkX = 78;
        }
        //drive to park.
        if (rings == Rings.ONE) {
            driveToPosition(36, 6, parkX, parkY, -90, 2, 1);
        } else if (rings == Rings.FOUR) {
            driveToPosition(new MotionProfile(12, 72,36),parkX, parkY, -90,1);

        } else {
            driveToPosition(36,6, parkX, bot.getPose().y + 3, -90,2,1);
        }

        bot.setShooterPower(0);

    }
}
