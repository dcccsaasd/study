package SACG_20minTiaoTime10;

import ilog.concert.*;
import ilog.cp.IloCP;
import ilog.cplex.IloCplex;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class InitialPattern {
    static int DataBase = 10;
    static double epson = 1e-6;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 20;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);

    static int MaxRouteLegNum = 10;
    static int MaxPatternLegNum = 12;

    static int[][] InitialPattern(int GateAirpID, int GateType, int NumberOfInitialPattern, int LegNum, int forcedCoveredCF,
                                  int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                  int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                  int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT,int[] CopyFlightArrT
    ) throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }

        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //强制覆盖FlightOrignIndex
        if (forcedCoveredCF != -1) {
            cp.addGe(cp.count(flightvars, forcedCoveredCF), 1);
        }

        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }

        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }

//        cp.solve();
//        System.out.println(cp.getStatus());
//        for(int i=0;i<maxlegs;i++){
//            if(cp.getValue(isactual[i])!=1){
//                break;
//            }
//                    System.out.print((int)cp.getValue(flightvars[i])+" ");
//        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,10);
        cp.startNewSearch();

        int[][] ActualPattern = new int[NumberOfInitialPattern][maxlegs+1];

        int findflag = 0;
        for (int n = 0; n < NumberOfInitialPattern; n++) {
            if (cp.next()) {
                findflag += 1;
                int numisactual = 0;
                for (int i = 0; i < maxlegs; i++) {
                    if (cp.getValue(isactual[i]) != 1) {
                        break;
                    }
                    numisactual +=1;
                    ActualPattern[n][i+1]=(int)cp.getValue(flightvars[i]);
                }
                ActualPattern[n][0] = numisactual;
//                System.out.println(Arrays.toString(ActualPattern[n]));
            }else{
                System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");

                cp.end();
                return ActualPattern;
            }
        }
        System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");
        cp.end();
        return ActualPattern;
    }


    static int[][] forcedCoveredPairInitialPattern(int GateAirpID, int GateType, int NumberOfInitialPattern, int LegNum, int forcedCoveredCF, int[] forcedCoveredPair,
                                                   int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                                   int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                                   int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT,int[] CopyFlightArrT
    ) throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }

        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //强制覆盖FlightOrignIndex
        if (forcedCoveredCF != -1) {
            cp.addGe(cp.count(flightvars, forcedCoveredCF), 1);
        }

        //强制覆盖forcedCoveredPair??
        IloIntExpr[] PairFlag = cp.intExprArray(maxlegs);
        PairFlag[0] = cp.prod(0, isactual[0]);
        for (int i = 1; i < maxlegs; i++) {
            PairFlag[i] = cp.prod(cp.eq(flightvars[i-1], forcedCoveredPair[0]), cp.eq(flightvars[i], forcedCoveredPair[1]));
            PairFlag[i] = cp.prod(PairFlag[i], isactual[i]);
        }
        cp.addGe(cp.sum(PairFlag),1);



        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }

        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }


//        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
//        cp.solve();
//        System.out.println(cp.getStatus());
//        for(int i=0;i<maxlegs;i++){
//            if(cp.getValue(isactual[i])!=1){
//                break;
//            }
//                    System.out.print((int)cp.getValue(flightvars[i])+" ");
//        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 12);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
//        cp.setParameter(IloCP.DoubleParam.TimeLimit,10);
        cp.startNewSearch();

        int[][] ActualPattern = new int[NumberOfInitialPattern][maxlegs+1];

        int findflag = 0;
        for (int n = 0; n < NumberOfInitialPattern; n++) {
            if (cp.next()) {
                findflag += 1;
                int numisactual = 0;
                for (int i = 0; i < maxlegs; i++) {
                    if (cp.getValue(isactual[i]) != 1) {
                        break;
                    }
                    numisactual +=1;
                    ActualPattern[n][i+1]=(int)cp.getValue(flightvars[i]);
                }
                ActualPattern[n][0] = numisactual;
                System.out.println(Arrays.toString(ActualPattern[n]));
            }else{
                System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");

                cp.end();
                return ActualPattern;
            }
        }
        System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");
        cp.end();
        return ActualPattern;
    }

    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException, IloException {
        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;
        int gateTypeNum = 2;
        int Initial1PatternNum = 1000;//1000
        int GateTypeDivideline = 0;
        if(DataBase==1){
            ActFlightNum = 24;
            MtsNum = 1;
            AircNum = 4;
            TypeNum = 2;
            ApNum = 7;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 1;
            }
        }else if(DataBase==2){
            ActFlightNum = 52;
            MtsNum = 1;
            AircNum = 8;
            TypeNum = 3;
            ApNum = 9;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 1;
            }
        }else if(DataBase==3){
            ActFlightNum = 98;
            MtsNum = 2;
            AircNum = 15;
            TypeNum = 3;
            ApNum = 14;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 1;
            }
        }else if(DataBase==4){
            ActFlightNum = 150;
            MtsNum = 2;
            AircNum = 24;
            TypeNum = 4;
            ApNum = 18;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 2;
            }
        }else if(DataBase==5){
            ActFlightNum = 198;
            MtsNum = 3;
            AircNum = 32;
            TypeNum = 4;
            ApNum = 17;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 2;
            }
        }else if(DataBase==6){
            ActFlightNum = 250;
            MtsNum = 3;
            AircNum = 41;
            TypeNum = 4;
            ApNum = 22;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 2;
            }
        }else if(DataBase==7){
            ActFlightNum = 295;
            MtsNum = 3;
            AircNum = 49;
            TypeNum = 5;
            ApNum = 26;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 2;
            }
        }else if(DataBase==8){
            ActFlightNum = 359;
            MtsNum = 3;
            AircNum = 61;
            TypeNum = 6;
            ApNum = 28;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 3;
            }
        }else if(DataBase==9){
            ActFlightNum = 395;
            MtsNum = 3;
            AircNum = 68;
            TypeNum = 8;
            ApNum = 30;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 4;
            }
        }else if(DataBase==10){
            ActFlightNum = 464;
            MtsNum = 3;
            AircNum = 81;
            TypeNum = 11;
            ApNum = 35;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 5;
            }
        }

        InputDataReader FlightReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\Flight.txt");
        int[][] Flight = FlightReader.readIntArrayArray();

        int FlightNum = Flight.length;

        int[] FlightID = new int[FlightNum];
        int[] FlightDepS = new int[FlightNum];
        int[] FlightArrS = new int[FlightNum];
        int[] FlightDepT = new int[FlightNum];
        int[] FlightArrT = new int[FlightNum];
        int[] FlightType = new int[FlightNum];
        int[] FlightOAID = new int[FlightNum];

        int[] FlightFlyT = new int[FlightNum];
        for(int f=0; f<FlightNum; f++){
            FlightID[f] = Flight[f][0];
            FlightDepS[f] = Flight[f][1];
            FlightArrS[f] = Flight[f][2];
            FlightDepT[f] = Flight[f][3];
            FlightArrT[f] = Flight[f][4];
            FlightType[f] = Flight[f][5];
            FlightOAID[f] = Flight[f][6];

            FlightFlyT[f] = FlightArrT[f] - FlightDepT[f];
        }


        int[][] CopyFlight = new int[ActFlightNum*CopyNum+MtsNum][7];
//        DataReader CopyFlightReader = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\CopyFlight.txt");
//        for (int i = 0; i < CopyFlight.length; i++) {
//            for (int j = 0; j < CopyFlight[0].length; j++){
//                CopyFlight[i][j] = CopyFlightReader.next();
//            }
//        }
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
        int CopyFlightNum = CopyFlight.length;
        int[] CopyFlightOFID = new int[CopyFlightNum];
        int[] CopyFlightDepS = new int[CopyFlightNum];
        int[] CopyFlightArrS = new int[CopyFlightNum];
        int[] CopyFlightDepT = new int[CopyFlightNum];
        int[] CopyFlightArrT = new int[CopyFlightNum];
        int[] CopyFlightType = new int[CopyFlightNum];
        int[] CopyFlightOAID = new int[CopyFlightNum];

        int[] CopyFlightDepTSlot = new int[CopyFlightNum];
        int[] CopyFlightArrTSlot = new int[CopyFlightNum];
        int[][] TypeCopyFlightRange= new int[TypeNum][2];
        for(int tp=0; tp<TypeNum; tp++){
            TypeCopyFlightRange[tp][0] = CopyFlight.length;
            TypeCopyFlightRange[tp][1] = 0;
        }

        for(int cf=0; cf<CopyFlightNum; cf++){
            CopyFlightOFID[cf] = CopyFlight[cf][0];
            CopyFlightDepS[cf] = CopyFlight[cf][1];
            CopyFlightArrS[cf] = CopyFlight[cf][2];
            CopyFlightDepT[cf] = CopyFlight[cf][3];
            CopyFlightArrT[cf] = CopyFlight[cf][4];
            CopyFlightType[cf] = CopyFlight[cf][5];
            CopyFlightOAID[cf] = CopyFlight[cf][6];

            CopyFlightDepTSlot[cf] = (int) CopyFlightDepT[cf]/60;
            CopyFlightArrTSlot[cf] = (int) CopyFlightArrT[cf]/60;
            if(cf<TypeCopyFlightRange[CopyFlightType[cf]][0]){
                TypeCopyFlightRange[CopyFlightType[cf]][0]=cf;
            }
            if(cf>TypeCopyFlightRange[CopyFlightType[cf]][1]){
                TypeCopyFlightRange[CopyFlightType[cf]][1]=cf;
            }
        }


        InputDataReader AircReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\Aircraft.txt");
        int[][] Aircraft = AircReader.readIntArrayArray();

        int[] AircID = new int[AircNum];
        int[] AircType = new int[AircNum];
        int[] AircStartS = new int[AircNum];
        int[] AircEndS = new int[AircNum];
        int[] AircTurnT = new int[AircNum];
        int[] AircMts = new int[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            AircID[airc] = Aircraft[airc][0];
            AircType[airc] = Aircraft[airc][1];
            AircStartS[airc] = Aircraft[airc][2];
            AircEndS[airc] = Aircraft[airc][3];
            AircTurnT[airc] = Aircraft[airc][4];
            AircMts[airc] = Aircraft[airc][5];
        }


        int[] AircRouteNum = new int[AircNum];
        int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];

        DataReader RouteNumR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialRouteNum1000.txt");
        DataReader RouteR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialRoute1000.txt");
        for (int airc = 0; airc < AircNum; airc++) {
            AircRouteNum[airc] = RouteNumR.next();
            AircLegnumDlytimeSwapnumRoute[airc] = new int[AircRouteNum[airc]][3+MaxRouteLegNum];
        }
        for (int airc = 0; airc < AircNum; airc++) {
            for (int i = 0; i < AircRouteNum[airc]; i++){
                for(int j=0; j<3+MaxRouteLegNum; j++){
                    AircLegnumDlytimeSwapnumRoute[airc][i][j] = RouteR.next();
                }
            }
        }

        int[][][] AirpTypeCFID = new int[ApNum][2][CopyFlightNum];
        int[][] AirpTypeCFNum = new int[ApNum][2];
        int[][] AirpTypeFNum = new int[ApNum][2];
        for(int ap=0; ap<ApNum; ap++){
            for(int cf=0; cf<CopyFlightNum; cf++){
                if((CopyFlightDepS[cf]==ap||CopyFlightArrS[cf]==ap) && CopyFlightType[cf]<GateTypeDivideline){
                    AirpTypeCFID[ap][0][AirpTypeCFNum[ap][0]]=cf;
                    AirpTypeCFNum[ap][0]+=1;
                }
                if((CopyFlightDepS[cf]==ap||CopyFlightArrS[cf]==ap) && CopyFlightType[cf]>=GateTypeDivideline){
                    AirpTypeCFID[ap][1][AirpTypeCFNum[ap][1]]=cf;
                    AirpTypeCFNum[ap][1]+=1;
                }
            }
            for(int f=0; f<FlightNum; f++){
                if((FlightDepS[f]==ap||FlightArrS[f]==ap) && FlightType[f]<GateTypeDivideline){
                    AirpTypeFNum[ap][0]+=1;
                }
                if((FlightDepS[f]==ap||FlightArrS[f]==ap) && FlightType[f]>=GateTypeDivideline){
                    AirpTypeFNum[ap][1]+=1;
                }
            }
        }
        for(int ap=0; ap<ApNum; ap++){
//            System.out.println(Arrays.toString(AirpTypeFNum[ap]));
//            System.out.println(Arrays.toString(AirpTypeCFNum[ap]));
        }


        int[][][][] AirpTypeLegnumPattern = new int[ApNum][2][100000][1+MaxPatternLegNum];
        int[][] AirpTypePatternNum = new int[ApNum][2];
        int[][] CopyFlightPatternCoverNum = new int[CopyFlightNum][2];

        //初始化1：为每个机场的两个类型的Gate分别找Initial1PatternNum个不同LegNum的Pattern

        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                if(AirpTypeCFNum[ap][tp]>0){
                    for(int LegNum=2; LegNum<=Math.min(AirpTypeFNum[ap][tp],MaxPatternLegNum); LegNum++){
                        int[][] AB;
                        AB = InitialPattern(ap, tp, Initial1PatternNum, LegNum, -1,
                                GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp], FlightType, FlightDepS, FlightArrS,
                                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT,CopyFlightArrT);
                        for (int i = 0; i < AB.length; i++) {
                            if (AB[i][0] != 0) {
                                for(int j=0; j<1; j++){
                                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
                                }
                                for(int j=1; j<1+AB[i][0]; j++){
                                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
                                    CopyFlightPatternCoverNum[AB[i][j]][CopyFlightDepS[AB[i][j]]==ap?0:1] +=1;
                                }
                                AirpTypePatternNum[ap][tp] += 1;
                            } else {
                                break;
                            }
                        }
                    }
                }
            }
        }

        System.out.println("CopyFlightPatternCoverNum0: ");
        for(int cf=0; cf<CopyFlightNum; cf++){
            for(int i=0; i<2; i++){
                if(CopyFlightPatternCoverNum[cf][i]==0){
                    System.out.println(cf+" DA-"+i);
                }
            }
        }

        //初始化2：为每个CopyFlight找只含单个Pattern（以确保可行解），每个初始化1未覆盖的CopyFlight的DA找Initial2PatternNum个Pattern
        int Initial2PatternNum = 1;//10
        for(int cf=0; cf<CopyFlightNum; cf++){
            for(int da=0; da<2; da++){
                int ap = ((da==0)?CopyFlightDepS[cf]:CopyFlightArrS[cf]);
                int GateType = (CopyFlightType[cf]<GateTypeDivideline?0:1);
                int[][] AB1;
                AB1 = InitialPattern(ap, GateType, 1, 1, cf,
                        GateTypeDivideline, AirpTypeCFID[ap][GateType], AirpTypeCFNum[ap][GateType],
                        FlightType, FlightDepS, FlightArrS,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
                for (int i = 0; i < AB1.length; i++) {
                    if (AB1[i][0] != 0) {
                        for(int j=0; j<1; j++){
                            AirpTypeLegnumPattern[ap][GateType][AirpTypePatternNum[ap][GateType]][j] = AB1[i][j];
                        }
                        for(int j=1; j<1+AB1[i][0]; j++){
                            AirpTypeLegnumPattern[ap][GateType][AirpTypePatternNum[ap][GateType]][j] = AB1[i][j];
                        }
                        AirpTypePatternNum[ap][GateType] += 1;
                    } else {
                        break;
                    }
                }
                if(CopyFlightPatternCoverNum[cf][da]==0){
                    System.out.println(cf+" DA-"+da);
                    int breakflag = 0;
                    for(int LegNum=2; LegNum<=Math.min(AirpTypeFNum[ap][GateType],MaxPatternLegNum); LegNum++) {
                        if(breakflag==1){
                            break;
                        }
                        int[][] AB;
                        AB = InitialPattern(ap, GateType, Initial2PatternNum, LegNum, cf,
                                GateTypeDivideline, AirpTypeCFID[ap][GateType], AirpTypeCFNum[ap][GateType],
                                FlightType, FlightDepS, FlightArrS,
                                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
                        for (int i = 0; i < AB.length; i++) {
                            if (AB[i][0] != 0) {
                                for(int j=0; j<1; j++){
                                    AirpTypeLegnumPattern[ap][GateType][AirpTypePatternNum[ap][GateType]][j] = AB[i][j];
                                }
                                for(int j=1; j<1+AB[i][0]; j++){
                                    AirpTypeLegnumPattern[ap][GateType][AirpTypePatternNum[ap][GateType]][j] = AB[i][j];
                                    CopyFlightPatternCoverNum[AB[i][j]][CopyFlightDepS[AB[i][j]]==ap?0:1] +=1;
                                }
                                AirpTypePatternNum[ap][GateType] += 1;
                            } else {
                                breakflag = 1;
                                break;
                            }
                        }
                    }
                }
            }
        }

        System.out.println("CopyFlightPatternCoverNum0: ");
        for(int cf=0; cf<CopyFlightNum; cf++){
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                for(int i=0; i<2; i++){
                    if(CopyFlightPatternCoverNum[cf][i]==0){
                        System.out.println(cf+" DA-"+i);
                    }
                }
            }else{
                if((CopyFlightPatternCoverNum[cf][0]+CopyFlightPatternCoverNum[cf][1])==0){
                    System.out.println(cf+" Mts");
                }
            }
        }

        //输出初始Pattern
        FileWriter InitialPatternNumFW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialPatternNum"+Initial1PatternNum+"~"+gateTypeNum+".txt");
        for(int ap=0; ap< ApNum; ap++){
            InitialPatternNumFW.write(AirpTypePatternNum[ap][0]+"\t"+AirpTypePatternNum[ap][1]+"\n");
        }
        InitialPatternNumFW.close();

        // AirpTypeLegnumPattern[ap][tp][AircTypePatternNum[ap][tp]][j]
        FileWriter InitialPatternFW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialPattern"+Initial1PatternNum+"~"+gateTypeNum+".txt");
        for(int ap=0; ap< ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                for (int i = 0; i < AirpTypePatternNum[ap][tp]; i++) {
                    for(int j = 0; j<1+MaxPatternLegNum-1; j++){
                        InitialPatternFW.write(AirpTypeLegnumPattern[ap][tp][i][j]+"\t");
                    }
                    InitialPatternFW.write(AirpTypeLegnumPattern[ap][tp][i][1+MaxPatternLegNum-1]+"\n");
                }
            }
        }
        InitialPatternFW.close();

//        int[] pair = {29, 75};
//        forcedCoveredPairInitialPattern(0, 0, 10, -1, -1, pair,
//        GateTypeDivideline, AirpTypeCFID[0][0], AirpTypeCFNum[0][0],
//        FlightType, FlightDepS, FlightArrS,
//        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT,CopyFlightArrT);









    }
}

