package org.firstinspires.ftc.teamcode.skybot.opmodes.unused;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotTeleOp;
import org.firstinspires.ftc.teamcode.skybot.SkyBot;
import org.firstinspires.ftc.teamcode.skybot.opmodes.SkyBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
@Disabled
@TeleOp(name = "SkyBot Teleop Bot Two", group = "Yes")
public class SkyBotTeleOpbottwo extends MechBotTeleOp {
    public SkyBot bot = new SkyBot(MechBot.MotorType.NeverestOrbital20, 31.5, 31.75, 31.75,
            BNO055Enhanced.AxesMap.YZX, BNO055Enhanced.AxesSign.PNN, 1);

    public int targetArmRotationTicks;

    //Variables to for automatic hand pitch contro
    boolean pitchControl = false;       //true if automatic control is enabled
    //Paired set of arm rotation ticks and pitch servo position to set hand pitch
    public int armLevelTicks;
    public float pitchLevelPos;

    public boolean dragServoUp = false;

    //Toggle to enable/disable hand pitch control
    ButtonToggle toggleA2 = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad2.a; } };

    // Toggle to raise/lower the foundation grabber
    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad1.a; } };

    //Resets hand pitch level when pressed then released
    ButtonToggle toggleB2 = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad2.b; }};

    @Override
    public void runLoggingOpmode(){
        bot.init(hardwareMap, 0.0f, true);
        super.setup(bot);
        gamepad2.setJoystickDeadzone(0.05f);

        targetArmRotationTicks = bot.armRotation.getCurrentPosition();
        armLevelTicks = targetArmRotationTicks;
        pitchLevelPos = (float) bot.armPitchServo.getPosition();

        bot.setDragServoDown();


        waitForStart();

        while(opModeIsActive()){
            doDriveControl();

            if(gamepad1.left_bumper){
                bot.setIntakePower(-SkyBotAutonomous.INTAKE_POWER);
            } else if (gamepad1.right_bumper){
                bot.setIntakePower(SkyBotAutonomous.INTAKE_POWER);
            } else {
                bot.setIntakePower(0.0f);
            }

            if(gamepad1.b) {
                //bot.armWheelServo.setPower(1.0);
                bot.armClawServo.setPosition(.55);
            } else if(gamepad1.x) {
                //bot.armWheelServo.setPower(-1.0);
                bot.armClawServo.setPosition(.15);
            } else {
                //bot.armWheelServo.setPower(0);
            }

            //Handle Arm Rotation: manual control if joystick is being used; otherwise
            //use proportionate control to hold previous position
            if(Math.abs(gamepad2.left_stick_y) < 0.05f) {
                bot.armRotation.setPower(-.00125 * (bot.armRotation.getCurrentPosition() - targetArmRotationTicks));
            } else {
                bot.armRotation.setPower(-gamepad2.left_stick_y / (gamepad2.left_stick_y > 0 ? 1.5 : 3));
                if(bot.armRotation.getCurrentPosition() < 0) {
                    targetArmRotationTicks = bot.armRotation.getCurrentPosition();
                } else {
                    targetArmRotationTicks = 0;
                }
                telemetry.addData("targetArmRotationTicks", targetArmRotationTicks);
            }

            bot.armExtension.setPower(-gamepad2.right_stick_y);

            if(gamepad2.right_bumper) {
                float position = (float)bot.armYawServo.getPosition() + 0.005f;
                if(position > 1) {
                    position = 1;
                }
                bot.armYawServo.setPosition(position);
            } else if(gamepad2.left_bumper) {
                float position = (float)bot.armYawServo.getPosition() - 0.005f;
                if(position < 0) {
                    position = 0;
                }
                bot.armYawServo.setPosition(position);
            }

            if (toggleA1.update()) {
                dragServoUp = !dragServoUp;
                if(dragServoUp) {
                    bot.setDragServoUp();
                } else {
                    bot.setDragServoDown();
                }
            }

            //Handle Wrist Pitch Control -- this may be in either automatic or manual mode

            if (toggleA2.update()){
                pitchControl = ! pitchControl;
            }

            if (toggleB2.update()){
                armLevelTicks = bot.armRotation.getCurrentPosition();
                pitchLevelPos = (float)bot.armPitchServo.getPosition();
            }

            telemetry.addData("PITCH CONTROL", pitchControl);

            if (pitchControl){
                float tickOffSet = bot.armRotation.getCurrentPosition() - armLevelTicks;
                float servoOffSet = - tickOffSet * bot.ARM_DEGREES_PER_TICK / bot.PITCH_SERVO_DEGREES_PER_UNIT;
                float newPitchServoPos = pitchLevelPos + servoOffSet;
                if (newPitchServoPos > 1) newPitchServoPos = 1;
                else if (newPitchServoPos < 0) newPitchServoPos = 0;
                bot.armPitchServo.setPosition(newPitchServoPos);
            } else {
                if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
                    float position = (float) bot.armPitchServo.getPosition() + 0.005f;
                    if (position > 1) {
                        position = 1;
                    }
                    bot.armPitchServo.setPosition(position);
                } else if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                    float position = (float) bot.armPitchServo.getPosition() - 0.005f;
                    if (position < 0) {
                        position = 0;
                    }
                    bot.armPitchServo.setPosition(position);
                }
            }

            telemetry.update();
        }
    }


}
