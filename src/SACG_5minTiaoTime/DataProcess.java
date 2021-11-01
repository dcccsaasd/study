package SACG_5minTiaoTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class DataProcess {
    static int DataBase = 10;
    static int OneCancelCost = 100;
    static int OneSwapCost = 10;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 5;
    static int MintDelayCost = 2;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);



    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException {
        InputDataReader FlightReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Flight.txt");
        int[][] Flight = FlightReader.readIntArrayArray();

        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;
        if(DataBase==1){
            ActFlightNum = 24;
            MtsNum = 1;
            AircNum = 4;
            TypeNum = 2;
            ApNum = 7;
        }else if(DataBase==2){
            ActFlightNum = 52;
            MtsNum = 1;
            AircNum = 8;
            TypeNum = 3;
            ApNum = 9;
        }else if(DataBase==3){
            ActFlightNum = 98;
            MtsNum = 2;
            AircNum = 15;
            TypeNum = 3;
            ApNum = 14;
        }else if(DataBase==4){
            ActFlightNum = 150;
            MtsNum = 2;
            AircNum = 24;
            TypeNum = 4;
            ApNum = 18;
        }else if(DataBase==5){
            ActFlightNum = 198;
            MtsNum = 3;
            AircNum = 32;
            TypeNum = 4;
            ApNum = 17;
        }else if(DataBase==6){
            ActFlightNum = 250;
            MtsNum = 3;
            AircNum = 41;
            TypeNum = 4;
            ApNum = 22;
        }else if(DataBase==7){
            ActFlightNum = 297;
            MtsNum = 3;
            AircNum = 50;
            TypeNum = 5;
            ApNum = 26;
        }else if(DataBase==8){
            ActFlightNum = 359;
            MtsNum = 3;
            AircNum = 61;
            TypeNum = 6;
            ApNum = 28;
        }else if(DataBase==9){
            ActFlightNum = 395;
            MtsNum = 3;
            AircNum = 68;
            TypeNum = 8;
            ApNum = 30;
        }else if(DataBase==10){
            ActFlightNum = 464;
            MtsNum = 3;
            AircNum = 81;
            TypeNum = 11;
            ApNum = 35;
        }



        //飞机首尾航班
        int[][] FeijiSE = new int[AircNum][2];
        for(int airc=0; airc<AircNum; airc++){
            for(int f=0; f<Flight.length; f++){
                if(Flight[f][1]!=Flight[f][2]){
                    if(Flight[f][6]==airc){
                        FeijiSE[airc][0]=f;
                        FeijiSE[airc][1]=f;
                        break;
                    }
                }
            }
        }
        for(int airc=0; airc<AircNum; airc++){
            for(int f=0; f<Flight.length; f++){
                if(Flight[f][1]!=Flight[f][2]){
                    if(Flight[f][6]==airc){
                        if(Flight[f][3]< Flight[FeijiSE[airc][0]][3]){
                            FeijiSE[airc][0]=f;
                        }
                        if(Flight[f][4]> Flight[FeijiSE[airc][0]][4]){
                            FeijiSE[airc][1]=f;
                        }
                    }
                }
            }
        }
        for(int airc=0; airc<AircNum; airc++){
            System.out.println(Flight[FeijiSE[airc][0]][1]+"\t"+Flight[FeijiSE[airc][1]][2]);
        }

        //正常数据机场容量
        int[][] ApSlotDep = new int[ApNum][SlotNum];
        int[][] ApSlotArr = new int[ApNum][SlotNum];

        for(int f=0; f<Flight.length; f++){
            ApSlotDep[Flight[f][1]][(int) Flight[f][3]/60] +=1;
            ApSlotArr[Flight[f][2]][(int) Flight[f][4]/60] +=1;
        }
        for(int ap=0; ap<ApNum; ap++){
            for(int t=0; t<SlotNum; t++){
//                System.out.print(ApSlotDep[ap][t]+"\t");
                System.out.print(ApSlotArr[ap][t]+"\t");
            }
            System.out.print("\n");
        }


        int[][] CopyFlight = new int[ActFlightNum*CopyNum+MtsNum][7];
        int CopyFlag=0;
        for(int f=0; f<Flight.length; f++){
            if(Flight[f][1]!=Flight[f][2]){
                for(int i=0; i<CopyNum; i++){
                    for(int j=0; j<7;j++)
                        CopyFlight[CopyFlag][j] = Flight[f][j];
                    CopyFlight[CopyFlag][3] = Flight[f][3] + OneDelayTime*i;
                    CopyFlight[CopyFlag][4] = Flight[f][4] + OneDelayTime*i;
                    CopyFlag += 1;
                }
            }else{
                for(int j=0; j<7;j++)
                    CopyFlight[CopyFlag][j] = Flight[f][j];
                CopyFlight[CopyFlag][3] = Flight[f][3];
                CopyFlight[CopyFlag][4] = Flight[f][4];
                CopyFlag += 1;
            }
        }

//        InputDataReader ApSlotDepReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\ApSlotDep.txt");
//        int[][] ApSlotDep = ApSlotDepReader.readIntArrayArray();
//
//        InputDataReader ApSlotArrReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\ApSlotArr.txt");
//        int[][] ApSlotArr = ApSlotArrReader.readIntArrayArray();

//        int[] CopyFlightOFID = new int[CopyFlight.length];
//        int[] CopyFlightDepS = new int[CopyFlight.length];
//        int[] CopyFlightArrS = new int[CopyFlight.length];
//        int[] CopyFlightDepT = new int[CopyFlight.length];
//        int[] CopyFlightArrT = new int[CopyFlight.length];
//        int[] CopyFlightType = new int[CopyFlight.length];
//        int[] CopyFlightOAN = new int[CopyFlight.length];
//        int[] CopyFlightFlyT = new int[CopyFlight.length];
//        int[] CopyFlightDlyMint = new int[CopyFlight.length];
//        int[][] TypeCopyFlightRange= new int[TypeNum][2];
//        int[] TypeMtsNum = new int[TypeNum];
//        for(int tp=0; tp<TypeNum; tp++){
//            TypeCopyFlightRange[tp][0] = CopyFlight.length;
//            TypeCopyFlightRange[tp][1] = 0;
//        }
//        for(int cf=0; cf<CopyFlight.length; cf++){
//            CopyFlightOFID[cf] = CopyFlight[cf][0];
//            CopyFlightDepS[cf] = CopyFlight[cf][1];
//            CopyFlightArrS[cf] = CopyFlight[cf][2];
//            CopyFlightDepT[cf] = CopyFlight[cf][3];
//            CopyFlightArrT[cf] = CopyFlight[cf][4];
//            CopyFlightType[cf] = CopyFlight[cf][5];
//            CopyFlightOAN[cf] = CopyFlight[cf][6];
//            if(cf<TypeCopyFlightRange[CopyFlightType[cf]][0]){
//                TypeCopyFlightRange[CopyFlightType[cf]][0]=cf;
//            }
//            if(cf>TypeCopyFlightRange[CopyFlightType[cf]][1]){
//                TypeCopyFlightRange[CopyFlightType[cf]][1]=cf;
//            }
//            CopyFlightFlyT[cf] = CopyFlightArrT[cf] - CopyFlightDepT[cf];
//            CopyFlightDlyMint[cf] = CopyFlightDepT[cf] - Flight[CopyFlightOFID[cf]][3];
//            if(CopyFlightDepS[cf]==CopyFlightArrS[cf]){
//                TypeMtsNum[CopyFlightType[cf]] += 1;
//            }
//        }
//
        FileWriter CfWrite = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\CopyFlight.txt");
        for(int cf=0; cf<CopyFlight.length; cf++) {
            for (int i = 0; i < CopyFlight[cf].length-1; i++) {
                CfWrite.write(CopyFlight[cf][i] + "\t");
            }
            CfWrite.write(CopyFlight[cf][CopyFlight[cf].length-1] + "\n");
        }
        CfWrite.close();
//
//        int[][] TypeMtsCFID = new int[TypeNum][];
//        for(int tp=0; tp<TypeNum; tp++){
//            TypeMtsCFID[tp] = new int[TypeMtsNum[tp]];
//            if(TypeMtsNum[tp]>0){
//                int flag = 0;
//                for(int cf=TypeCopyFlightRange[tp][0]; cf<=TypeCopyFlightRange[tp][1]; cf++){
//                    if(CopyFlightDepS[cf]==CopyFlightArrS[cf]){
//                        TypeMtsCFID[tp][flag]=cf;
//                        flag+=1;
//                        if(flag==TypeMtsNum[tp])
//                            break;
//                    }
//                }
//            }
//        }
//        InputDataReader AircReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Aircraft.txt");
//        int[][] Aircraft = AircReader.readIntArrayArray();
//
//
//        int[] AircID = new int[Aircraft.length];
//        int[] AircType = new int[Aircraft.length];
//        int[] AircStartS = new int[Aircraft.length];
//        int[] AircEndS = new int[Aircraft.length];
//        int[] AircTurnT = new int[Aircraft.length];
//        int[] AircMts = new int[Aircraft.length];
//
//        for(int airc=0; airc<Aircraft.length; airc++){
//            AircID[airc] = Aircraft[airc][0];
//            AircType[airc] = Aircraft[airc][1];
//            AircStartS[airc] = Aircraft[airc][2];
//            AircEndS[airc] = Aircraft[airc][3];
//            AircTurnT[airc] = Aircraft[airc][4];
//            AircMts[airc] = Aircraft[airc][5];
//        }
//
//







    }
}

