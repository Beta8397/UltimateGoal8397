package org.firstinspires.ftc.teamcode.goalbot;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "GoalBotTeleOp", group = "GoalBot")
public class GoalbotTeleop extends MecBotTeleOp {
    GoalBot bot= new GoalBot();

    private GoalBot.IntakeState intakeState = GoalBot.IntakeState.OFF;

    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle toggleA2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.a;
        }
    };

    private boolean grabberClosed = true;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);

        waitForStart();

        while (opModeIsActive()) {
            handleIntake();
            float armPower = -0.25f * gamepad2.left_stick_y;
            if (toggleA2.update()){
                grabberClosed = !grabberClosed;
//                bot.grabber.setPosition    TODO: Handle grabber state
            }
            bot.setArmPower(armPower);
            doDriveControl();
            telemetry.update();

        }
    }

    private void handleIntake() {

        boolean aToggled = toggleA1.update();
        switch (intakeState) {
            case OFF:
                if (gamepad1.b) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (aToggled){
                    intakeState = GoalBot.IntakeState.FWD;
                }
                break;
            case FWD:
                if (gamepad1.b) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (aToggled) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
            case REV:
                if (!gamepad1.b) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
        }
        bot.setIntake(intakeState);

    }
}
