package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "servoTest")
public class servoTest extends LinearOpMode{

    public Servo servo;


    double servoPos = 0.025;

    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(servoPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                servoPos += 0.0005;
            }
            if (gamepad1.right_bumper) {
                servoPos -= 0.0005;
                servoPos = Math.max(servoPos, 0.025);
            }
            servo.setPosition(servoPos);

            telemetry.addData("serPos", "%.4f", servoPos);
            telemetry.addData("Servo Real Pos", "%.4f", servo.getPosition());
            telemetry.update();
        }

//        telemetry.addData("Stuff", "Stuff");
//        telemetry.update();

    }
}
