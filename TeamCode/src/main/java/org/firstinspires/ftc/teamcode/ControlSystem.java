package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.params.BlackLevelPattern;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by arno on 18-5-21.
 */
@Disabled
public class ControlSystem extends BasicOpMode_Linear
{
    //region 定义闸门舵机
    Servo servoGate;
    Servo servoBlue;
    Servo servoOrange;
    //endregion
    //region 闸门角度
    double BluePosition;
    double OrangePosition;

    //endregion
    public void ControlSystemConfigue()
    {
        //region 实例化闸门舵机
        servoGate = hardwareMap.get(Servo.class,"servoGate");
        servoBlue = hardwareMap.get(Servo.class,"servoBlue");
        servoOrange = hardwareMap.get(Servo.class,"servoOrange");
        //endregion
        //region 初始化闸门角度
        BluePosition = servoBlue.getPosition();
        OrangePosition = servoOrange.getPosition();
        //endregion
    }

    public double[] SetDegree()
    {
        double[] returnData = new double[2];
        if(gamepad1.dpad_down==true||gamepad1.dpad_up==true||gamepad1.dpad_left==true||gamepad1.dpad_right==true)
        {

            //region 根据手柄1的方向轮盘调整红蓝球门的角度
            if(gamepad1.dpad_up==true)
            {
                if(BluePosition+.02 <= 1)
                {
                    BluePosition = BluePosition+0.02;
                } else
                {
                    BluePosition = 1;
                }
                sleep(50);
            }
            if(gamepad1.dpad_down==true)
            {
                if(BluePosition-.02 >= 0)
                {
                    BluePosition = BluePosition-0.02;
                } else
                {
                    BluePosition = 0;
                }
                sleep(50);
            }
            if(gamepad1.dpad_left==true)
            {
                if(OrangePosition+.02 <= 1)
                {
                    OrangePosition = OrangePosition+0.02;
                } else
                {
                    OrangePosition = 1;
                }
                sleep(50);
            }

            if(gamepad1.dpad_right==true)
            {
                if(OrangePosition-.02 >= 0)
                {
                    OrangePosition = OrangePosition-0.02;
                } else
                {
                    OrangePosition = 0;
                }
                sleep(50);
            }
            //endregion

            returnData[0] = BluePosition;
            returnData[1] = OrangePosition;
        }
        else
        {
            returnData[0]=0;
            returnData[1]=0;
        }
        servoOrange.setPosition(OrangePosition);
        servoBlue.setPosition(OrangePosition);

        return returnData;
    }
}
