package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;


public class LimeCam {

    public final Limelight3A limelight;
    public boolean [] tagList=new boolean[16];
    public boolean tagDetected (int id){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()){
                if (fid.getFiducialId() == id){
                    return true;
                }
            }
            return false;

        } else {
            return false;
        }
    }

    public boolean [] checkTagList (){
        int i=30; //April Tag ID starts at 30
        for (int j=0; j<16; j++){
            tagList[j]=false;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                for (int j = 0; j < 16; j++) {
                    if (fid.getFiducialId() == i + j) {
                        tagList[j] = true;

                    }
                }
            }
        }
        return tagList;
    }

    public boolean seeRedFront (){// red front means red side hive near audience
        checkTagList();
        boolean result=false;
        for (int i=0; i<4; i++){
            if (tagList[i+4]) result=true;
        }

        return result;
    }

    public boolean seeRedBack (){ //red back means red side hive opposite of audience
        checkTagList();
        boolean result=false;
        for (int i=0; i<4; i++){
            if (tagList[i]) result=true;
        }

        return result;
    }

    public boolean seeBlueBack (){ //red back means blue side hive opposite of audience
        checkTagList();
        boolean result=false;
        for (int i=0; i<4; i++){
            if (tagList[i+12]) result=true;
        }

        return result;
    }

    public boolean seeBlueFront (){ //red back means blue side hive near audience
        checkTagList();
        boolean result=false;
        for (int i=0; i<4; i++){
            if (tagList[i+8]) result=true;
        }

        return result;
    }
    public LimeCam(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();


    }
}
