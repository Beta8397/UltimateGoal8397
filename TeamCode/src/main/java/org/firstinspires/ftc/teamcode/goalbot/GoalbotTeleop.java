package org.firstinspires.ftc.teamcode.goalbot;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "GoalBotTeleOp", group = "GoalBot")
public class GoalbotTeleop extends MecBotTeleOp {

    public static final float SHOOTER_DELAYED = 1.0f;

    GoalBot bot= new GoalBot();

    private GoalBot.IntakeState intakeState = GoalBot.IntakeState.OFF;

    ButtonToggle toggleRightBumper1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.right_bumper;
        }
    };

    ButtonToggle toggleA2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.a;
        }
    };

    ButtonToggle toggleRightBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.right_bumper;
        }
    };

    ButtonToggle toggleLeftBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.left_bumper;
        }
    };

    ButtonToggle toggleB2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.b;
        }
    };
    ButtonToggle toggleX2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() { return gamepad2.x; }
    };
    ButtonToggle toggleDpadDown2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.dpad_down;
        }
    };
    ButtonToggle toggleDpadUp2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.dpad_up;
        }
    };

    private boolean grabberClosed = true;
    private boolean shooterOn = false;
    private boolean shooterHigh = false;
    private boolean shooting = false;
    private boolean ringStuck = true;
    private ElapsedTime shootingTimer= new ElapsedTime();
    private enum ArmPos{
        IN, UP, OUT
    }

    private ArmPos armPos = ArmPos.IN;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);
        bot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        float armPosition = bot.getArmTargetPosition();

        waitForStart();

        while (opModeIsActive()) {
            handleIntake();
            if (toggleA2.update()){
                grabberClosed = !grabberClosed;
                if (grabberClosed) {
                    bot.setGrabberClosed();
                } else {
                    bot.setGrabberOpen();
                }
//                bot.grabber.setPosition    TODO: Handle grabber state
            }

            boolean dUp = toggleDpadUp2.update();
            boolean dDown = toggleDpadDown2.update();
            boolean armStateChange = false;

            if (armPos == ArmPos.IN){
                if (dDown){
                    armPos = ArmPos.OUT;
                    armStateChange = true;
                }

            } else {
                if (dDown){
                    armPos = ArmPos.IN;
                    armStateChange = true;
                } else {
                    if(dUp){
                        armPos = armPos == ArmPos.OUT? ArmPos.UP:ArmPos.OUT;
                        armStateChange = true;
                    }
                }
            }
            if(armStateChange) {

                int armTarget = armPos == ArmPos.OUT ? 590 :
                        armPos == ArmPos.UP ? 300 : 80;
                bot.setArmPosition(armTarget);
            }
            if(armPos == ArmPos.IN && bot.getArmActualPos() < 100){
                bot.setArmPower(0);
            }


            if (toggleRightBumper2.update()) {
                shooterOn = !shooterOn;
            }

            if (toggleLeftBumper2.update()) {
                shooterHigh = !shooterHigh;
            }
            if (shooterOn) {
                if (shooterHigh) {
                    bot.setShooterPowerHigh();
                } else {
                    bot.setShooterPowerNormal();
                }
            } else {
                bot.setShooterPower(0);
            }

            if (toggleX2.update()) {
                ringStuck = !ringStuck;
            }
            if (ringStuck) {
                bot.setRingKickerEngaged();
            } else {
                bot.setRingKickerUnengaged();
            }


            float shooterRPM = (float)bot.shooter.getVelocity(AngleUnit.DEGREES) / 6;
            telemetry.addData("shooter_RPM", shooterRPM);

            if (toggleB2.update()) {
                shooting = true;
                shootingTimer.reset();
            } else if(!gamepad2.b) {
                shooting = false;
            }

            if (shooting) {
                if (shootingTimer.seconds() < .5 * SHOOTER_DELAYED) {
                    bot.setKickerEngaged();
                } else if (shootingTimer.seconds() < SHOOTER_DELAYED) {
                    bot.setKickerUnengaged();
                } else {
                    shootingTimer.reset();
                }
            } else {
                bot.setKickerUnengaged();
            }



            doDriveControl();
            telemetry.update();

        }
    }

    private void handleIntake() {

        boolean RightBumperToggled = toggleRightBumper1.update();
        switch (intakeState) {
            case OFF:
                if (gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (RightBumperToggled){
                    intakeState = GoalBot.IntakeState.FWD;
                }
                break;
            case FWD:
                if (gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (RightBumperToggled) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
            case REV:
                if (!gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;

        }
        bot.setIntake(intakeState);

    }
}
