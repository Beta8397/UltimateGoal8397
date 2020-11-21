package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TestIntake4;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

public class GoalBot extends MecBot {

    public static final float GRABBER_OPEN_POSITION = 0.2f;
    public static final float GRABBER_CLOSED_POSITION = 0.7f;
    public static final float SHOOTER_POWER_NORMAL = 0.75f;
    public static final float SHOOTER_POWER_HIGH = 1.0f;
    public static final float KICKER_ENGAGED = 0;
    public static final float KICKER_UNENGAGED = 0.19f;

    DcMotorEx intakeFront;
    DcMotorEx intakeBack;
    public DcMotorEx armMotor;
    DcMotorEx shooter;
    Servo kicker;
    Servo grabber;
    public enum IntakeState {
        OFF, FWD, REV
    }

    public GoalBot(){
        super(MotorType.NeverestOrbital20, 13.25f, 13, 4, 40.9f, 1, BNO055Enhanced.AxesMap.XZY,
                BNO055Enhanced.AxesSign.NNN);
    }


    public void init(HardwareMap hwMap){
        super.init(hwMap);
        intakeFront = hwMap.get(DcMotorEx.class, "intake front");
        intakeBack = hwMap.get(DcMotorEx.class, "intake back");
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber = hwMap.get(Servo.class, "grabber");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE );
        kicker = hwMap.get(Servo.class, "kicker");
    }

    public void setArmMode(DcMotor.RunMode Mode){
        if (Mode == DcMotor.RunMode.RUN_TO_POSITION) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
        }
        armMotor.setMode(Mode);
    }

    public void setArmPosition(int ticks){
        armMotor.setTargetPosition(ticks);
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armMotor.setPower(.3);
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

    public void setShooterPower(float pwr) {
        shooter.setPower(pwr);
    }

    public void setShooterPowerNormal() {
        setShooterPower(SHOOTER_POWER_NORMAL);
    }

    public void setShooterPowerHigh() {
        setShooterPower(SHOOTER_POWER_HIGH);
    }

    public void setKickerPosition(float pos) {
        kicker.setPosition(pos);
    }

    public void setKickerEngaged() {
        setKickerPosition(KICKER_ENGAGED);
    }

    public void setKickerUnengaged() {
        setKickerPosition(KICKER_UNENGAGED);
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
