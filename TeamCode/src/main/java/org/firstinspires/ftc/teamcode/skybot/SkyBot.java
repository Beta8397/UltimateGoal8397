package org.firstinspires.ftc.teamcode.skybot;

import com.qualcomm.robotcore.hardware.CRServo;
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
public class SkyBot extends MechBot {

    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;
    public DcMotor armRotation = null;
    public DcMotor armExtension = null;
    public DistanceSensor frontDistSensor = null;
    public Servo dragServo = null;
    public Servo armClawServo = null;
    public Servo armYawServo = null;
    public Servo armPitchServo = null;
    public Servo capstonePlacerFlip;
    public Servo capstonePlacerGrip;

    //Up and down positions for the foundation-dragging servo
    public static final float DRAG_SERVO_DOWN = 0.80f;
    public static final float DRAG_SERVO_UP = 0.5f;

    // When the flip servo is on the stone and when it's up
    public static final float FLIP_SERVO_FLIP = .1f;
    // I was strongly tempted to call this FLIP_SERVO_FLOP
    public static final float FLIP_SERVO_BACK = .45f;

    // Grip and release positions for capstoneServoGrip
    public static final float GRIP_SERVO_GRIP = .2f;
    public static final float GRIP_SERVO_RELEASE = .1f;

    //Conversion factors for the arm rotation motor and the arm pitch servo
    public static final float ARM_DEGREES_PER_TICK = -360.0f / (28.0f * 256.0f);
    public static final float ARM_CM_PER_TICK = 20.0f / 1120.0f;
    public static final float PITCH_SERVO_DEGREES_PER_UNIT = 200.0f;

    public static final double PITCH_SERVO_MAX_POS = 0.9;
    public static final double PITCH_SERVO_MIN_POS = 0.0;

    public static final float CLAW_SERVO_OPEN = .55f;
    public static final float CLAW_SERVO_CLOSED = .15f;


    public SkyBot(MotorType motorType, double wheelCirc, double length, double width,
                  BNO055Enhanced.AxesMap axMap, BNO055Enhanced.AxesSign axSign, double gearRatio) {
        super(motorType, wheelCirc, length, width, axMap, axSign, gearRatio);
    }


    public void init(HardwareMap ahwMap, float initHeadingDegrees, boolean initializeIMU){
        super.init(ahwMap, initHeadingDegrees, initializeIMU);

        //Intake motors
        intakeLeft = ahwMap.dcMotor.get("Intake LM");
        intakeRight = ahwMap.dcMotor.get("Intake RM");
        armRotation = ahwMap.dcMotor.get("arm rotation");
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtension = ahwMap.dcMotor.get("arm extension");
        frontDistSensor = ahwMap.get(DistanceSensor.class, "distSens");
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        dragServo = ahwMap.servo.get("dragServo");
        armClawServo = ahwMap.servo.get("claw servo");
        armYawServo = ahwMap.servo.get("yaw servo");
        armPitchServo = ahwMap.servo.get("pitch servo");
        armPitchServo.scaleRange(PITCH_SERVO_MIN_POS, PITCH_SERVO_MAX_POS);
        capstonePlacerFlip = ahwMap.servo.get("capstonePlacerFlip");
        capstonePlacerGrip = ahwMap.servo.get("capstonePlacerGrip");
    }

    public void setIntakePower(float intakePow){
        intakeLeft.setPower(intakePow);
        intakeRight.setPower(intakePow);
    }

    public void setDragServoDown() {
        dragServo.setPosition(DRAG_SERVO_DOWN);
    }

    public void setDragServoUp() {
        dragServo.setPosition(DRAG_SERVO_UP);
    }

    public void setFlipServoDown() { capstonePlacerFlip.setPosition(FLIP_SERVO_FLIP);}

    public void setFlipServoUp() { capstonePlacerFlip.setPosition(FLIP_SERVO_BACK); }

    public void setGripServoGripped() { capstonePlacerGrip.setPosition(GRIP_SERVO_GRIP); }

    public void setGripServoReleased() { capstonePlacerGrip.setPosition(GRIP_SERVO_RELEASE); }

}
