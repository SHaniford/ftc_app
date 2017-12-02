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

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TOBORRedSideAutonomous", group="Autonomous")
//@Disabled
public class ToborRedSideAutonomous extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
    TOBORVuMarkIdentification.Crypto CryptoColumn;
    public enum state{
        DRIVEOFFSTONE,
        SEEKCOLUMN,
        MOVEFORWARD,
        PLACEBLOCK,
        BACKUP

    }
    //double TPI = 1120/(4*Math.PI);
    int TPI = 1493;
    state currentState = state.DRIVEOFFSTONE;
    @Override
    public void init()
    {

        telemetry.addData("Status", "Initialized");
        robo.initRobo(hardwareMap);
        robo.initVuforia();
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void init_loop()
    {
        robo.seekImage();
        if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Left)
        {
            telemetry.addData("Vumark Left","Acquired");
            CryptoColumn = TOBORVuMarkIdentification.Crypto.Left;
        }
        else if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Center)
        {
            telemetry.addData("Vumark Center","Acquired");
            CryptoColumn = TOBORVuMarkIdentification.Crypto.Center;
        }
        else if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Right)
        {
            telemetry.addData("Vumark Right","Acquired");
            CryptoColumn = TOBORVuMarkIdentification.Crypto.Right;
        }
        else
        {
            telemetry.addData("Vumark", "Unknown");
        }
        robo.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (currentState == state.DRIVEOFFSTONE)
        {
            robo.BRMotor.setTargetPosition(-24*TPI);
            robo.BLMotor.setTargetPosition(24*TPI);
            robo.FRMotor.setTargetPosition(24*TPI);
            robo.FLMotor.setTargetPosition(-24*TPI);
            robo.strafeLeft(1);
        }
        else if (currentState == state.SEEKCOLUMN)
        {
            if (CryptoColumn == TOBORVuMarkIdentification.Crypto.Left)
            {

            }
            else if (CryptoColumn == TOBORVuMarkIdentification.Crypto.Right)
            {

            }
            else if (CryptoColumn == TOBORVuMarkIdentification.Crypto.Center)
            {

            }
            else
            {

            }
        }

    }

    @Override
    public void stop()
    {

    }

}
