package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.LEFT_FLIPPO_DOWN;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.LEFT_FLIPPO_FINGER_CLOSED;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.LEFT_FLIPPO_FINGER_OPEN;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.LEFT_FLIPPO_UP;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.RIGHT_FLIPPO_DOWN;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.RIGHT_FLIPPO_FINGER_CLOSED;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.RIGHT_FLIPPO_FINGER_OPEN;
import static org.firstinspires.ftc.teamcode.newbot2bot.NewBot2Bot.RIGHT_FLIPPO_UP;

/**
 * Created by FTC Team 8397 on 11/5/2019.
 */
@TeleOp(name="Test Servo", group="Test")
public class TestServo extends LinearOpMode {
    String[] servoNames = new String[]{"capServo", "armGrabber", "leftFlippo", "leftFlippoFinger", "rightFlippo", "rightFlippoFinger", "dragServo"};
    Servo[] servos;
    float[] servoPositions;
    int activeServo = 0;

    //Toggle to switch servos
    ButtonToggle toggleX = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad1.x; } };

    public void runOpMode() {
        DistanceSensor frontDist =  hardwareMap.get(DistanceSensor.class, "distSens");
        DistanceSensor backDist =  hardwareMap.get(DistanceSensor.class, "backDistSens");

        servos = new Servo[servoNames.length];
        servoPositions = new float[servoNames.length];
        for(int i = 0; i < servoNames.length; i++) {
            servos[i] = hardwareMap.servo.get(servoNames[i]);
        }

        waitForStart();

        while(opModeIsActive()) {
            if(toggleX.update()) {
                activeServo = (activeServo + 1) % servos.length;
            }

            if(gamepad1.y) {
                servoPositions[activeServo] += .005;
            } else if(gamepad1.a) {
                servoPositions[activeServo] -= .005;
            }

            if(gamepad1.dpad_up) {
                servoPositions[activeServo] += .015;
            } else if(gamepad1.dpad_down) {
                servoPositions[activeServo] -= .015;
            }

            if(servoPositions[activeServo] < 0) {
                servoPositions[activeServo] = 0;
            }

            if(servoPositions[activeServo] > 1) {
                servoPositions[activeServo] = 1;
            }

            servos[activeServo].setPosition(servoPositions[activeServo]);

            for(int i = 0; i < servos.length; i++) {
                telemetry.addData(servoNames[i], servoPositions[i]);
            }
            telemetry.addData("Active Servo", servoNames[activeServo]);
            telemetry.addData("Btns", "X: toggle, Dup/Ddn: fast, A/Y: slow");

            telemetry.addData("Front Distance", frontDist.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", backDist.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}
