package org.firstinspires.ftc.teamcode;

public class MM_Autos extends MM_OpMode{
    public static int SPLINE_DETAIL_LEVEL = 20;

    private enum STATES {
        SCORE
    }

    private STATES state = STATES.SCORE;
    @Override
    public void runProcedures(){
        while (opModeIsActive()){
            switch(state) {
                case SCORE:
                    //TODO add code XD


            }
        }
    }
}
