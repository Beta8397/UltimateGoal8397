package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo", group = "test")
public class TestServo extends LinearOpMode {

    Servo kicker;

    public void runOpMode() {
        kicker = hardwareMap.get(Servo.class, "front_gate");
        waitForStart();
        float pos = 0;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                pos = pos + 0.01f;
            } else if (gamepad1.b) {
                pos = pos - 0.01f;
            }

            if (pos < 0) {
                pos = 0;
            } else if (pos > 1) {
                pos = 1;
            }

            kicker.setPosition(pos);
            telemetry.addData("position", pos);
            telemetry.update();
        }
    }

}
