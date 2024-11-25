package org.firstinspires.ftc.teamcode.FromOldRepo.Call_Upon_Classes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Color_Sensor {
    private RevColorSensorV3 test;
    private int redness;
    private int greenness;
    private boolean isred;
    private boolean isgreen;


    public void init_Color(HardwareMap hardwareMap, String name) {
        test = hardwareMap.get(RevColorSensorV3.class, name);
    }


      public void run_Color() {

          redness=test.red();
          if (redness > 155) {
              isred = true;
          } else {
              isred = false;
          }

          redness=test.green();
          if (greenness > 155) {
              isred = true;
          } else {
              isred = false;
          }
    }

        public void get_Telemetry (Telemetry telemetry){
            telemetry.addData("Color Red", test.red());
            telemetry.addData("Color Blue", test.blue());
            telemetry.addData("Color Green", test.green());
            telemetry.addData("is red", isred);

        }
    }

