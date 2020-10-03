package org.firstinspires.ftc.teamcode.newbot2bot.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotTeleOp;
import org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;
import org.firstinspires.ftc.teamcode.skybot.opmodes.SkyBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@TeleOp(name = "NewBot2Bot Teleop", group = "Yes")
public class NewBot2BotTeleOp extends MechBotTeleOp {
    public NewBot2Bot bot = new NewBot2Bot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.XZY, BNO055Enhanced.AxesSign.PNP, 1);

    private float targetArmHeightTicks = 0;

    private int liftPosition = -1;

    private float[] armHeights = new float[]{0, -225, -375, -525, -675, -825, -975, -1125};

    private float flippoPosition = 1;

    private boolean foundationGrabberUp = false;

    public boolean dragServoUp = false;

    private boolean capServoIsOpen = false;

    private int retractArmStep = -1;

    private ElapsedTime retractArmTimer = new ElapsedTime();


    // Toggle to raise/lower the foundation grabber
    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad1.a; } };

    //Toggle to open/close the cap servo
    ButtonToggle toggleDPadUp2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.dpad_up; }
    };

    ButtonToggle toggleY2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.y; }
    };

    ButtonToggle toggleDPadRight2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.dpad_right; }
    };

    ElapsedTime toggleDPadRight2Timer = new ElapsedTime();

    ButtonToggle toggleDPadLeft2Press = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.dpad_left; }
    };

    ButtonToggle toggleDPadLeft2Release = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad2.dpad_left; }
    };


    @Override
    public void runLoggingOpmode(){
        ElapsedTime et = new ElapsedTime();
        bot.init(hardwareMap, 0.0f, true);
        BetaLog.dd("bot.init() ", String.valueOf(et.milliseconds()));
        super.setup(bot);
        BetaLog.dd("super.setup() ", String.valueOf(et.milliseconds()));
        gamepad2.setJoystickDeadzone(JOYSTICK_DEADZONE);
        gamepad1.setJoystickDeadzone(JOYSTICK_DEADZONE);

        //targetArmHeightTicks = bot.liftyThingLeft.getCurrentPosition();

        //bot.setDragServoDown();
        telemetry.addData("Lifter Pos", "L: %d  R: %d", bot.liftyThingLeft.getCurrentPosition(), bot.liftyThingRight.getCurrentPosition());
        telemetry.update();
        BetaLog.dd("deadzones and telemetry ", String.valueOf(et.milliseconds()));
        bot.setCapServoClosed();
        BetaLog.dd("bot.setCapServoClosed() ", String.valueOf(et.milliseconds()));

        waitForStart();

        bot.setRightFlippoUp();
        bot.setLeftFlippoUp();
        bot.setRightFlippoFingerClosed();
        bot.setLeftFlippoFingerClosed();

        while(opModeIsActive()) {
            doDriveControl();

            if (gamepad1.left_bumper) {
                bot.setIntakePower(-SkyBotAutonomous.INTAKE_POWER);
            } else if (gamepad1.right_bumper) {
                bot.setIntakePower(SkyBotAutonomous.INTAKE_POWER);
            } else {
                bot.setIntakePower(0);
            }

            //Handle Foundation Grabber
            if (toggleA1.update()) {
                if (foundationGrabberUp) {
                    bot.setDragServoDown();
                } else {
                    bot.setDragServoUp();
                }
                foundationGrabberUp = !foundationGrabberUp;
            }

            //Handle Arm Grab Servo
            if (gamepad2.x) {
                bot.setGrabServoClosed();
            } else if (gamepad2.b) {
                bot.setGrabServoFullOpen();
            } else if (gamepad2.a) {
                bot.setGrabServoOpen();
            }

            //Handle Cap Servo
            if (toggleDPadUp2.update()) {
                if (capServoIsOpen) bot.setCapServoClosed();
                else bot.setCapServoOpen();
                capServoIsOpen = !capServoIsOpen;
            }
            float rightTrigger = gamepad2.right_trigger < TRIGGER_DEADZONE ? 0 : gamepad2.right_trigger;
            float leftTrigger = gamepad2.left_trigger < TRIGGER_DEADZONE ? 0 : gamepad2.left_trigger;
            bot.rightFlippo.setPosition(flippoPosition += (rightTrigger - leftTrigger) * .02);
            telemetry.addData("Flippo", "Pos: %.2f  RT: %.2f  LT: %.2f", flippoPosition, rightTrigger, leftTrigger);

            if (gamepad2.left_bumper)
                bot.setRightFlippoFingerClosed();
            else if (gamepad2.right_bumper)
                bot.setRightFlippoFingerOpen();



            if (gamepad1.x)
                bot.setBlockPusherClosed();
            if (gamepad1.b)
                bot.setBlockPusherOpen();

            // Handle Arm Height: manual control if joystick is being used; otherwise
            // use proportionate control to hold previous position
            // The position of the arm at the start of this opMode is defined as 0 ticks
            //To avoid excessive calls to getCurrentPosition(), call once and pass through to bot methods

            float gp2LeftY = gamepad2.left_stick_y;

            float leftTicks = bot.liftyThingLeft.getCurrentPosition();
            float rightTicks = bot.liftyThingRight.getCurrentPosition();
            float avgTicks = (leftTicks + rightTicks) / 2.0f;

            if (Math.abs(gp2LeftY) < 0.05) {
                    bot.updateLiftPosition(targetArmHeightTicks);
            } else {
                if (gp2LeftY > 0) {
                    bot.setLiftPower(gp2LeftY / 4, leftTicks, rightTicks);  //Positive power drives down

                } else {
                    bot.setLiftPower(gp2LeftY / 1, leftTicks, rightTicks);     //Negative power drives up
                }
                targetArmHeightTicks = avgTicks;
            }

            if(toggleY2.update()) {
                if (!gamepad2.dpad_right && liftPosition < (armHeights.length-1))  liftPosition++;
                if (liftPosition < 0) liftPosition = 0;
                targetArmHeightTicks = armHeights[liftPosition];
            }

            if (toggleDPadRight2.update()){
                if (toggleDPadRight2Timer.milliseconds() < 500) {
                    liftPosition = 0;
                    targetArmHeightTicks = armHeights[liftPosition];
                }
                toggleDPadRight2Timer.reset();
            }

            float sldPwr = -gamepad2.right_stick_y * 0.5f;

            if(toggleDPadLeft2Press.update()) {
                retractArmStep = 0;
                retractArmTimer.reset();
                bot.fastLiftMode = true;
            }

            if(toggleDPadLeft2Release.update()) {
                bot.setGrabServoOpen();
                bot.fastLiftMode = false;
            }

            telemetry.addData("Retract Arm", "Step: %d  Timer: %d", retractArmStep, (int)retractArmTimer.milliseconds());

            if(gamepad2.dpad_left) {
                if(retractArmTimer.milliseconds() > 100 && retractArmStep >= 0) {
                    retractArmStep++;
                    retractArmTimer.reset();
                }
                if(retractArmStep <= 1) {
                    bot.setGrabServoFullOpen();
                    if(liftPosition < armHeights.length - 1)
                        targetArmHeightTicks = armHeights[liftPosition + 1];
                    else
                        targetArmHeightTicks = armHeights[liftPosition] - 100;
                } else if(retractArmStep <= 7) {
                    sldPwr = -.5f;
                } else {
                    sldPwr = -.5f;
                    targetArmHeightTicks = armHeights[0];
                    bot.setGrabServoClosed();
                }
            } else {
                retractArmStep = -1;
            }

            telemetry.addData("Lift Ticks", "Lt %.0f  Rt %.0f  Avg %.0f", leftTicks, rightTicks, avgTicks);
            telemetry.addData("Auto Lift", "TgtTicks %.0f  LftPos %d", targetArmHeightTicks, liftPosition);

            telemetry.addData("Setting slider to", sldPwr);
            bot.setArmExtenderPower(sldPwr);


            telemetry.addData("dragServoUp", dragServoUp);
            telemetry.addData("Touch Pressed?", bot.getTouchPressed());

            telemetry.update();
        }
    }
}



