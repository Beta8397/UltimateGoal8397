package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.goalbot.GoalBot;
import org.firstinspires.ftc.teamcode.goalbot.GoalBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "testStack", group = "red")
public class TestStack extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 69;
    public static final float Y_SHOOT = 50;
    public static final float angle1 = -153;
    public static final float angle2 = -160;
    public static final float angle3 = -166;

    GoalBot bot = new GoalBot();

    GoalBotAutonomous.Rings rings = Rings.ZERO;
    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    public void runLoggingOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        VuforiaNavigator.activate(null, null);
        sleep(3000);
        CameraDevice.getInstance().setField("zoom", ""+20);
        VuforiaNavigator.setFlashTorchMode(true);
        waitForStart();
        while (opModeIsActive()) {
            if (toggleA1.update()) {
                rings = getRings(true);
                telemetry.addData("rings ", rings);
                telemetry.update();
            }
        }
        VuforiaNavigator.setFlashTorchMode(false);
    }
}
