package org.firstinspires.ftc.teamcode.mechbot;

import android.annotation.SuppressLint;
import android.content.Context;
import android.media.MediaPlayer;
import android.speech.tts.TextToSpeech;
import android.speech.tts.Voice;
import android.util.Log;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Locale;

/**
 * Abstract class to provide TeleOp functionality for a MechBot.
 *
 * Note:  Subclasses of MechBotTeleOp must call MechBotTeleOp.setup(...), to provide MechBotTeleOp
 * with a reference to a MechBot, and do other initialization.
 *
 * It is the responsibility of the subclass to initialize the MechBot.
 *
 * The control loop of the subclass of MechBotTeleOp should call MechBotTeleOp.doDriveControl()
 * during each iteration.
 *
 */
@TeleOp(name = "Speak", group = "qwertyuiop")
public class MechBotTeleOpSpeak extends LoggingLinearOpMode {

    private TextToSpeech tts;
    private ButtonToggle speakToggle = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad1.back;
        }
    };

    /**
     * Handle one iteration of MechBot drive control.
     * Checks toggle buttons for slow mode, quad mode, and telemetry
     * Checks left joystick and triggers, then sets robot drive powers (x,y,angle)
     * If telemetry enabled, adds drive data to telemetry
     */
    public void runLoggingOpmode(){
        try {
            @SuppressLint("PrivateApi") final Class<?> activityThreadClass = Class.forName("android.app.ActivityThread");
            final Method method = activityThreadClass.getMethod("currentApplication");
            final Context c = (Context) method.invoke(null, (Object[]) null);
            tts = new TextToSpeech(c, new TextToSpeech.OnInitListener() {
                @Override
                public void onInit(int status) {
                    if (status == TextToSpeech.SUCCESS) {
                        int ttsLang = tts.setLanguage(Locale.US);

                        if (ttsLang == TextToSpeech.LANG_MISSING_DATA
                                || ttsLang == TextToSpeech.LANG_NOT_SUPPORTED) {
                            Log.e("TTS", "The Language is not supported!");
                        } else {
                            Log.i("TTS", "Language Supported.");
                        }
                        Log.i("TTS", "Initialization success.");
                    } else {
                        Toast.makeText(c, "TTS Initialization failed!", Toast.LENGTH_SHORT).show();
                    }
                }
            });
        } catch (InvocationTargetException | NoSuchMethodException | IllegalAccessException | ClassNotFoundException e) {
            e.printStackTrace();
        }
        waitForStart();
        while(opModeIsActive()) {
            if (speakToggle.update()) {
                int speechStatus = tts.speak("Thank you for your time. We would be thankful to answer any questions you have.", TextToSpeech.QUEUE_FLUSH, null);
                if (speechStatus == TextToSpeech.ERROR) {
                    Log.e("TTS", "Error in converting Text to Speech!");
                }
                stop();
            }
        }
    }


}
