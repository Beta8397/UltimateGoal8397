package org.firstinspires.ftc.teamcode.skybot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
public class TempBot extends MechBot {


    public DistanceSensor frontDistSensor = null;
    public DistanceSensor backDistSensor = null;
    public DistanceSensor leftDistSensor = null;
    public Servo dragServo = null;

    //Up and down positions for the foundation-dragging servo
    public static final float DRAG_SERVO_DOWN = 0.80f;
    public static final float DRAG_SERVO_UP = 0.5f;

    public TempBot(MotorType motorType, double wheelCirc, double length, double width,
                   BNO055Enhanced.AxesMap axMap, BNO055Enhanced.AxesSign axSign, double gearRatio) {
        super(motorType, wheelCirc, length, width, axMap, axSign, gearRatio);
    }


    public void init(HardwareMap ahwMap, float initHeadingDegrees, boolean initializeIMU){
        super.init(ahwMap, initHeadingDegrees, initializeIMU);

        frontDistSensor = ahwMap.get(DistanceSensor.class, "distSens");
        backDistSensor = ahwMap.get(DistanceSensor.class, "backDistSens");
        leftDistSensor = ahwMap.get(DistanceSensor.class, "leftDistSens");
        dragServo = ahwMap.servo.get("dragServo");

    }

    public void setDragServoDown() {
        dragServo.setPosition(DRAG_SERVO_DOWN);
    }

    public void setDragServoUp() {
        dragServo.setPosition(DRAG_SERVO_UP);
    }

}
