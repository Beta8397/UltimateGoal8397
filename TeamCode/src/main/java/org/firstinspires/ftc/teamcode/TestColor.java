 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous (name = "test color", group = "test")
public class TestColor extends LinearOpMode {

    ColorSensor colorSensor;

    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();

        while(opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            telemetry.addData("color", "R = %d G = %d B = %d", red, green, blue);
            telemetry.update();
        }
    }

}
