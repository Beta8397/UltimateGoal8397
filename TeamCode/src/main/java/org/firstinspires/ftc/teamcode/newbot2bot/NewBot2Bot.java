package org.firstinspires.ftc.teamcode.newbot2bot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;

/**
 * Created by FTC Team 8397 on 10/29/2019.
 */
public class NewBot2Bot extends MechBot {
    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;
    public DcMotor liftyThingLeft = null;
    public DcMotor liftyThingRight = null;

    public DistanceSensor frontDistSensor = null;
    public DistanceSensor backDistSensor = null;
    public DistanceSensor leftDistSensor = null;
    public DistanceSensor rightDistSensor = null;

    public DigitalChannel touchSensor = null;

    public Servo dragServo = null;
    public Servo armGrabber = null;
    public Servo rightFlippo = null;
    public Servo rightFlippoFinger = null;
    public Servo leftFlippo = null;
    public Servo leftFlippoFinger = null;
    public Servo blockPusher = null;

    public CRServo armExtender = null;

    public Servo capServo = null;

    //Up and down positions for the foundation-dragging servo
    public static final float DRAG_SERVO_DOWN = 0.82f;
    public static final float DRAG_SERVO_UP = 0.5f;
    public static final float GRAB_SERVO_OPEN = 0.75f;
    public static final float GRAB_SERVO_FULL_OPEN = 0.65f;
    public static final float GRAB_SERVO_CLOSED = 0.9f;
    public static final float RIGHT_FLIPPO_UP = 0.33f;
    public static final float RIGHT_FLIPPO_DOWN = 0.76f;
    public static final float RIGHT_FLIPPO_FINGER_OPEN = 0f;
    public static final float RIGHT_FLIPPO_FINGER_CLOSED = 0.68f;
    public static final float LEFT_FLIPPO_UP = 0.78f;
    public static final float LEFT_FLIPPO_DOWN = 0.35f;
    public static final float LEFT_FLIPPO_FINGER_OPEN = 0.75f;
    public static final float LEFT_FLIPPO_FINGER_CLOSED = 0.13f;
    public static final float CAP_SERVO_OPEN = 0.2f;
    public static final float CAP_SERVO_CLOSED = 0.31f;
    public static final float BLOCK_PUSHER_OPEN = 0;
    public static final float BLOCK_PUSHER_CLOSED =0.5f;


    private final float LIFT_CONTROL_COEFFICIENT = 1/400f;
    private final float LIFT_CONTROL_COEFFICIENT_2 = 1/400f;
    private final float LIFT_CONTROL_COEFFICIENT_FAST = 1.5f/400f;

    public boolean fastLiftMode = false;

    public NewBot2Bot(MotorType motorType, double wheelCirc, double length, double width,
                      BNO055Enhanced.AxesMap axMap, BNO055Enhanced.AxesSign axSign, double gearRatio) {
        super(motorType, wheelCirc, length, width, axMap, axSign, gearRatio);
    }


    public void init(HardwareMap ahwMap, float initHeadingDegrees, boolean initializeIMU){
        super.init(ahwMap, initHeadingDegrees, initializeIMU);

        //Intake motors
        intakeLeft = ahwMap.dcMotor.get("intakeLeft");
        intakeRight = ahwMap.dcMotor.get("intakeRight");

        // Arm raiser
        liftyThingLeft = ahwMap.dcMotor.get("lifterLeft");
        liftyThingRight = ahwMap.dcMotor.get("lifterRight");
        liftyThingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftyThingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftyThingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftyThingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftyThingRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtender = ahwMap.crservo.get("armExtender");
        armExtender.setDirection(DcMotorSimple.Direction.FORWARD);

        frontDistSensor = ahwMap.get(DistanceSensor.class, "distSens");
        backDistSensor = ahwMap.get(DistanceSensor.class, "backDistSens");
        leftDistSensor = ahwMap.get(DistanceSensor.class, "leftDist");
        rightDistSensor = ahwMap.get(DistanceSensor.class, "rightDist");


        touchSensor = ahwMap.digitalChannel.get("touchSensor");

        dragServo = ahwMap.servo.get("dragServo");
        dragServo.setDirection(Servo.Direction.REVERSE);
        dragServo.scaleRange(DRAG_SERVO_UP, DRAG_SERVO_DOWN);
        //armGrabber = ahwMap.servo.get("armGrabber");

        leftFlippo = ahwMap.servo.get("leftFlippo");
        rightFlippo = ahwMap.servo.get("rightFlippo");
        leftFlippoFinger = ahwMap.servo.get("leftFlippoFinger");
        rightFlippoFinger = ahwMap.servo.get("rightFlippoFinger");

        leftFlippo.scaleRange(LEFT_FLIPPO_DOWN, LEFT_FLIPPO_UP);
        leftFlippoFinger.scaleRange(LEFT_FLIPPO_FINGER_CLOSED, LEFT_FLIPPO_FINGER_OPEN);

        rightFlippo.setDirection(Servo.Direction.REVERSE);
        rightFlippoFinger.setDirection(Servo.Direction.REVERSE);
        rightFlippo.scaleRange(RIGHT_FLIPPO_UP, RIGHT_FLIPPO_DOWN);
        rightFlippoFinger.scaleRange(RIGHT_FLIPPO_FINGER_OPEN, RIGHT_FLIPPO_FINGER_CLOSED);

        armGrabber = ahwMap.servo.get("armGrabber");
        capServo = ahwMap.servo.get("capServo");
        blockPusher = ahwMap.servo.get("blockPusher");

    }

    //NOTE: getState() returns false when sensor is pressed, so return the logical inverse
    public boolean getTouchPressed() { return !touchSensor.getState(); }

    public void setIntakePower(float intakePow){
        intakeLeft.setPower(intakePow);
        intakeRight.setPower(-intakePow);
    }

    public void updateLiftPosition(float targetArmPosition){
        float leftTicks = liftyThingLeft.getCurrentPosition();
        float rightTicks = liftyThingRight.getCurrentPosition();
        float avgPosition = (leftTicks + rightTicks)/2.0f;

        float pAvg = LIFT_CONTROL_COEFFICIENT * (targetArmPosition - avgPosition);
        if(fastLiftMode) {
            pAvg = LIFT_CONTROL_COEFFICIENT_FAST * (targetArmPosition - avgPosition);
        }
        if (pAvg > 0) pAvg /= 4.0f;
        pAvg = Math.max(-1, Math.min(1, pAvg));
        setLiftPower(pAvg, leftTicks, rightTicks, true);
    }

    public void setLiftPower(float power, float leftTicks, float rightTicks) {
        setLiftPower(power, leftTicks, rightTicks, false);
    }

    public void setLiftPower(float power, float leftTicks, float rightTicks, boolean hardFloorLimit){
        float posDifference = leftTicks - rightTicks;
        float pRight = power + LIFT_CONTROL_COEFFICIENT_2 * posDifference;
        float pLeft = power - LIFT_CONTROL_COEFFICIENT_2 * posDifference;
        float max = Math.max(Math.abs(pRight), Math.abs(pLeft));
        if(max > 1) {
            pRight /= max;
            pLeft /= max;
        }
        //Code to prevent constant motor activity when lift is all the way down and in auto control
        //If either lift has ticks > 35, that lift is all the way down, so don't apply any downward (positive) power
        if (hardFloorLimit && leftTicks > -10) pLeft = Math.min(0, pLeft);
        if (hardFloorLimit && rightTicks > -10) pRight = Math.min(0, pRight);
        liftyThingLeft.setPower(pLeft);
        liftyThingRight.setPower(pRight);
    }

    public float getAverageLifterPosition() {
        return (liftyThingLeft.getCurrentPosition() + liftyThingRight.getCurrentPosition()) / 2f;
    }

    public void setArmExtenderPower(float power) {
        armExtender.setPower((float)Math.max(-0.9, Math.min(0.9, power)));
    }

    public void setDragServoDown() {
        dragServo.setPosition(0);
    }

    public void setDragServoUp() {
        dragServo.setPosition(1);
    }

    public void setGrabServoOpen() {armGrabber.setPosition(GRAB_SERVO_OPEN);}

    public void setGrabServoClosed() {armGrabber.setPosition(GRAB_SERVO_CLOSED);}

    public void setGrabServoFullOpen() {armGrabber.setPosition(GRAB_SERVO_FULL_OPEN);}

    public void setRightFlippoUp() {rightFlippo.setPosition(1);}

    public void setRightFlippoDown() {rightFlippo.setPosition(0);}

    public void setRightFlippoFingerOpen() {rightFlippoFinger.setPosition(1);}

    public void setRightFlippoFingerClosed() {rightFlippoFinger.setPosition(0);}

    public void setLeftFlippoUp() {leftFlippo.setPosition(1);}

    public void setLeftFlippoDown() {leftFlippo.setPosition(0);}

    public void setLeftFlippoFingerOpen() {leftFlippoFinger.setPosition(1);}

    public void setLeftFlippoFingerClosed() {leftFlippoFinger.setPosition(0);}

    public void setCapServoClosed(){capServo.setPosition(CAP_SERVO_CLOSED);}

    public void setCapServoOpen(){capServo.setPosition(CAP_SERVO_OPEN);}
    public void setBlockPusherOpen(){blockPusher.setPosition(BLOCK_PUSHER_OPEN);}
    public void setBlockPusherClosed(){blockPusher.setPosition(BLOCK_PUSHER_CLOSED);}


}
