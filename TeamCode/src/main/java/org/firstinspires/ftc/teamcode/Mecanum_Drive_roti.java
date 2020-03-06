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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Muie so multa vLaD", group="Pushbot")

public class Mecanum_Drive_roti extends OpMode {

    /* Declare OpMode members. */
     HardwareConfig robot = new HardwareConfig();
     double coeff = 1;
    boolean highSpeed= true;
    double LB,LF,RF,RB;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        robot.glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if( gamepad1.start && gamepad1.a ){
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
        }

        double drive;
        double turn;
        double strafe;

        drive  = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn   = gamepad1.right_stick_x;


        LB = -(drive - strafe + turn);
        LF = -(drive + strafe + turn);
        RB = -(drive + strafe - turn);
        RF = -(drive - strafe - turn);


        LB*=coeff;
        LF*=coeff;
        RB*=coeff;
        RF*=coeff;


        if(gamepad1.x){
            highSpeed = false;
        }else if(gamepad1.b){
            highSpeed = true;
        }

        if( !highSpeed ){
            lower_power( 0.3 );
        }

        robot.leftBack.setPower(LB);
        robot.leftFront.setPower(LF);
        robot.rightFront.setPower(RF);
        robot.rightBack.setPower(RB);

        // suptere
        if(gamepad2.left_trigger > 0.1){
            robot.suptStanga.setPower(1);
            robot.suptDreapta.setPower(-1);
        }else if(gamepad2.right_trigger > 0.1){
            robot.suptStanga.setPower(-1);
            robot.suptDreapta.setPower(1);
        }else{
            robot.suptStanga.setPower(0);
            robot.suptDreapta.setPower(0);
        }

        //ghitu e gay
        //muie a/3
        //ridicat glisiere
        if(gamepad2.dpad_up){
            robot.glisiere.setPower(1);
        }else if(gamepad2.dpad_down){
            robot.glisiere.setPower(-0.5);
        }else{
            robot.glisiere.setPower(0.1);
            //robot.glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(gamepad2.x){
            robot.intoarcere.setPosition(0.9);
        }else if(gamepad2.b){
            robot.intoarcere.setPosition(0.13);
        }

        if(gamepad2.a){
            robot.prins.setPosition(0.65);
        }else if(gamepad2.y){
            robot.prins.setPosition(1);
        }

        //tras placa
        if(gamepad1.a){
            robot.prins_fundatie_dr.setPosition(0.75);
            robot.prins_fundatie_stg.setPosition(0);
        }else if(gamepad1.y){
            robot.prins_fundatie_dr.setPosition(0);
            robot.prins_fundatie_stg.setPosition(0.65);
        }

        //pentru 2 miscari deodata
        /*
        if(gamepad2.dpad_right){
            robot.prins.setPosition(0.8);
            sleep(50);
            robot.intoarcere.setPosition(0.7);
            sleep(50);
        }else if(gamepad2.dpad_left){
            robot.prins.setPosition(0);
            sleep(50);
            robot.intoarcere.setPosition(0);
        }  */


        // Use gamepad buttons to move the arm up (Y) and down (A)

        // Send telemetry message to signify robot running;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void lower_power(double gain){
        LB *= gain;
        LF *= gain;
        RB *= gain;
        RF *= gain;
    }
    private void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){e.printStackTrace();}
    }

}
