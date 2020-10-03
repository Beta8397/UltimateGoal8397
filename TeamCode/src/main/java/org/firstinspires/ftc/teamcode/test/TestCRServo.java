package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "TestCRServo", group = "Test")
public class TestCRServo extends LinearOpMode {

    private CRServo crServo;

    public void runOpMode(){

        crServo = hardwareMap.crservo.get("crServo");

        waitForStart();

        while (opModeIsActive()){

            float p = -gamepad1.left_stick_y;
            p = Math.max(-0.8f, Math.min(0.8f, p));
            crServo.setPower(p);

            telemetry.addData("Power: ", " %.2f", p);
            telemetry.update();
        }


    }

}
