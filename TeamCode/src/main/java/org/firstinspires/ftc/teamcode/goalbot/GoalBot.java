package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestIntake4;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

public class GoalBot extends MecBot {

    public static final float GRABBER_OPEN_POSITION = 0.2f;
    public static final float GRABBER_CLOSED_POSITION = 0.7f;

    DcMotor intakeFront;
    DcMotor intakeBack;
    DcMotor armMotor;
    Servo grabber;
    enum IntakeState {
        OFF, FWD, REV
    }

    public GoalBot(){
        super(MotorType.NeverestOrbital20, 13.25f, 13, 4, 43, 1, BNO055Enhanced.AxesMap.XZY,
                BNO055Enhanced.AxesSign.NNN);
    }


    public void init(HardwareMap hwMap){
        super.init(hwMap);
        intakeFront = hwMap.get(DcMotor.class, "intake front");
        intakeBack = hwMap.get(DcMotor.class, "intake back");
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hwMap.get(DcMotor.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        grabber = hwMap.get(Servo.class, "grabber");
    }

    public void setIntake(IntakeState intakeState) {
        if (intakeState == IntakeState.OFF) {
            setIntakePower(0);
        } else if (intakeState == IntakeState.FWD) {
            setIntakePower(1);
        } else {
            setIntakePower(-1);
        }
    }

    public void setArmPower(float pwr){
        armMotor.setPower(pwr);
    }

    public void setGrabberPosition(float pos){
        grabber.setPosition(pos);
    }

    public void setGrabberOpen(){
        grabber.setPosition(GRABBER_OPEN_POSITION);
    }

    public void setGrabberClosed(){
        grabber.setPosition(GRABBER_CLOSED_POSITION);
    }

    public void setIntakePower(float p){
        intakeFront.setPower(p);
        intakeBack.setPower(p);
    }
}
