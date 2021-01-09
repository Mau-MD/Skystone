/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Prueba Wobble", group="Test")
public class PruebaGarra extends LinearOpMode {

    DcMotor brazo;
    Servo codo;
    Servo mano;

    ElapsedTime dpadArriba = new ElapsedTime();
    ElapsedTime dpadAbajo = new ElapsedTime();
    ElapsedTime dpadIzquierda = new ElapsedTime();
    ElapsedTime dpadDerecha = new ElapsedTime();
    ElapsedTime botonA = new ElapsedTime();
    ElapsedTime botonB = new ElapsedTime();

    @Override
    public void runOpMode() {

        dpadArriba.reset();
        dpadAbajo.reset();
        dpadIzquierda.reset();
        dpadDerecha.reset();
        botonA.reset();
        botonB.reset();

        brazo = hardwareMap.dcMotor.get("br");
        codo = hardwareMap.servo.get("cd");
        mano = hardwareMap.servo.get("mn");

        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double codoPos = 0;
        double manoPos = 0;

        int positionGoal = 0;
        while (opModeIsActive()) {

            double brazoPower = 0.0;

            if (gamepad1.right_trigger > 0.0) {
                brazoPower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0.0) {
                brazoPower = gamepad1.left_trigger;
            }

            if (gamepad1.a && botonA.milliseconds() > 300)
            {
                positionGoal += 100;
                botonA.reset();
            }

            if (gamepad1.b && botonB.milliseconds() > 300)
            {
                positionGoal -= 100;
                botonB.reset();
            }

            if (gamepad1.dpad_up && dpadArriba.milliseconds() > 300) {
                codoPos += 0.1;
                dpadArriba.reset();
            }
            else if (gamepad1.dpad_down && dpadAbajo.milliseconds() > 300)
            {
                codoPos -= 0.1;
                dpadAbajo.reset();
            }

            if (gamepad1.dpad_right && dpadDerecha.milliseconds() > 300)
            {
                manoPos += 0.1;
                dpadDerecha.reset();
            }
            else if (gamepad1.dpad_left && dpadIzquierda.milliseconds() > 300)
            {
                manoPos -= 0.1;
                dpadIzquierda.reset();
            }


            int error = positionGoal - brazo.getCurrentPosition();
            brazoPower = error * 0.01;

            codoPos = Range.clip(codoPos,0.0,1.0);
            manoPos = Range.clip(manoPos, 0.0, 1.0);
            brazoPower = Range.clip(brazoPower, -1.0, 1.0);





            telemetry.addData("Codo Pos: ", codoPos);
            telemetry.addData("Mano Pos: ", manoPos);
            telemetry.addData("Brazo Power: ", brazoPower);
            telemetry.addData("Error: ", error);
            telemetry.update();

            codo.setPosition(codoPos);
            mano.setPosition(manoPos);
            brazo.setPower(brazoPower);


        }
    }
}
