package crewpairing;

import ilog.concert.*;
import ilog.cplex.*;
import ilog.cp.*;

import java.io.*;

public class crewpairing {
    private static class DataReader {

        private StreamTokenizer st;

        public DataReader(String filename) throws IOException { //throws IOException 抛出异常
            FileInputStream fstream = new FileInputStream(filename);
            Reader r = new BufferedReader(new InputStreamReader(fstream));
            st = new StreamTokenizer(r);
        }

        public int next() throws IOException {
            st.nextToken();
            return (int) st.nval;
        }
    }


    static int[][] onepairing(int[] flightsdeps, int[] flightsarrs, int[] flightsdept, int[] flightsarrt,
                           int[] flightsflytime, int[] flightstsmin, int[] flightstsmax, int latestArrivalTime, int forcedCoveredFlight) throws  IloException{

        int[][] CostActualPairing = new int[NumberOfInitialPairings][10];

        int flightsnum = flightsdeps.length;
        IloCP cp = new IloCP();

        IloIntVar[] flightvars = cp.intVarArray(maxlegs,0, flightsnum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs,0, 1);

        IloIntVar starttime = cp.intVar(0, latestArrivalTime);
        IloIntVar endtime = cp.intVar(0, latestArrivalTime);
        IloIntVar worktime = cp.intVar(0, latestArrivalTime);
        IloIntVar flytime = cp.intVar(0, latestArrivalTime);

        IloIntVar cost = cp.intVar(0, IloCP.IntMax);
        IloIntVar worktimecost = cp.intVar(0, IloCP.IntMax);
        IloIntVar flytimecost = cp.intVar(0, IloCP.IntMax);

        cp.add(cp.ge(cp.sum(isactual), 2));

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        cp.add(cp.eq( cp.element(flightsdeps, flightvars[0]) , cp.element(flightsarrs, flightvars[maxlegs-1])));

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(flightsdeps, flightvars[i]) , cp.element(flightsarrs, flightvars[i-1]) ) ));
        }

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(flightsdept, flightvars[i]) ,
                            cp.sum(cp.element(flightsarrt, flightvars[i-1]), cp.element(flightstsmin,flightvars[i])) ) ));
        }

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.le( cp.element(flightsdept, flightvars[i]) ,
                            cp.sum(cp.element(flightsarrt, flightvars[i-1]), cp.element(flightstsmax,flightvars[i])) ) ));
        }

        cp.add( cp.ge(cp.count(flightvars, forcedCoveredFlight),1));

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }

        IloIntExpr flytimeactual = cp.prod(cp.element(flightsflytime, flightvars[0]), isactual[0]);
        for(int i=1; i<maxlegs; i++){
            flytimeactual = cp.sum(flytimeactual, cp.prod(cp.element(flightsflytime, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( flytime, flytimeactual ));

        cp.add( cp.eq(starttime, cp.element(flightsdept, flightvars[0])));
        cp.add( cp.eq(endtime, cp.element(flightsarrt, flightvars[maxlegs-1])));
        cp.add( cp.eq(worktime, cp.diff(endtime, starttime)));

        cp.add( cp.le(flytime, maxflytime));
        cp.add( cp.le(worktime, maxworktime));

        cp.add( cp.eq(flytimecost, cp.prod(flytime, flypayrate)));
        cp.add( cp.eq(worktimecost, cp.prod(worktime, workpayrate)));
        cp.add( cp.eq(cost, cp.max(minpay, cp.max(flytimecost, worktimecost))));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);//减速
        cp.setParameter(IloCP.IntParam.LogPeriod, 1);
        cp.startNewSearch();
        for(int n=0; n<NumberOfInitialPairings; n++){
            cp.next();
            CostActualPairing[n][0] = (int) cp.getValue(cost);
            int numisactual = 0;
            for(int i=0;i<maxlegs;i++){
                if(cp.getValue(isactual[i])!=1){
                    break;
                }
                numisactual +=1;
                CostActualPairing[n][i+2]=(int)cp.getValue(flightvars[i]);
//                System.out.print((int)cp.getValue(flightvars[i])+" ");
            }
            CostActualPairing[n][1] = numisactual;
        }
//        for(int n =0; n<NumberOfInitialPairings; n++){
//            System.out.println(Arrays.toString(CostActualPairing[n]));
//        }

        return CostActualPairing;
    }


    static double[] onebestpairing(int[] flightsdeps, int[] flightsarrs, int[] flightsdept, int[] flightsarrt,
                              int[] flightsflytime, int[] flightstsmin, int[] flightstsmax, int latestArrivalTime, double[] flightsdual) throws  IloException{

        double[] ReducedcostCostActualPairing = new double[11];

        int flightsnum = flightsdeps.length;
        IloCP cp = new IloCP();

        IloIntVar[] flightvars = cp.intVarArray(maxlegs,0, flightsnum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs,0, 1);

        IloIntVar starttime = cp.intVar(0, latestArrivalTime);
        IloIntVar endtime = cp.intVar(0, latestArrivalTime);
        IloIntVar worktime = cp.intVar(0, latestArrivalTime);
        IloIntVar flytime = cp.intVar(0, latestArrivalTime);

        IloIntVar cost = cp.intVar(0, IloCP.IntMax);
        IloIntVar worktimecost = cp.intVar(0, IloCP.IntMax);
        IloIntVar flytimecost = cp.intVar(0, IloCP.IntMax);

        IloNumExpr obj = cost;
        for(int i=0; i<maxlegs; i++){
            obj = cp.diff(obj,
                          cp.prod( cp.element(flightsdual, flightvars[i]), cp.addGe(isactual[i], 1) )
                    );
        }
        cp.addMinimize(obj);


        cp.add(cp.ge(cp.sum(isactual), 2));

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        cp.add(cp.eq( cp.element(flightsdeps, flightvars[0]) , cp.element(flightsarrs, flightvars[maxlegs-1])));

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(flightsdeps, flightvars[i]) , cp.element(flightsarrs, flightvars[i-1]) ) ));
        }

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(flightsdept, flightvars[i]) ,
                            cp.sum(cp.element(flightsarrt, flightvars[i-1]), cp.element(flightstsmin,flightvars[i])) ) ));
        }

        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.le( cp.element(flightsdept, flightvars[i]) ,
                            cp.sum(cp.element(flightsarrt, flightvars[i-1]), cp.element(flightstsmax,flightvars[i])) ) ));
        }


        for(int i = 1; i<maxlegs; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }

        IloIntExpr flytimeactual = cp.prod(cp.element(flightsflytime, flightvars[0]), isactual[0]);
        for(int i=1; i<maxlegs; i++){
            flytimeactual = cp.sum(flytimeactual, cp.prod(cp.element(flightsflytime, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( flytime, flytimeactual ));

        cp.add( cp.eq(starttime, cp.element(flightsdept, flightvars[0])));
        cp.add( cp.eq(endtime, cp.element(flightsarrt, flightvars[maxlegs-1])));
        cp.add( cp.eq(worktime, cp.diff(endtime, starttime)));

        cp.add( cp.le(flytime, maxflytime));
        cp.add( cp.le(worktime, maxworktime));

        cp.add( cp.eq(flytimecost, cp.prod(flytime, flypayrate)));
        cp.add( cp.eq(worktimecost, cp.prod(worktime, workpayrate)));
        cp.add( cp.eq(cost, cp.max(minpay, cp.max(flytimecost, worktimecost))));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.solve();

        ReducedcostCostActualPairing[0] = cp.getObjValue();
        ReducedcostCostActualPairing[1] = cp.getValue(cost);
        int numisactual = 0;
        for(int i=0;i<maxlegs;i++){
            if(cp.getValue(isactual[i])!=1){
                break;
            }
            numisactual +=1;
            ReducedcostCostActualPairing[i+3]=cp.getValue(flightvars[i]);
        }
        ReducedcostCostActualPairing[2] = numisactual;
//        System.out.println(Arrays.toString(CostActualPairing));

        return ReducedcostCostActualPairing;
    }


    static double[] LP(int[][] CostLegnumPairing, int PairingNum, int FlightNum) throws IloException {

        IloCplex lpcplex = new IloCplex();

        IloNumVar[] p = lpcplex.numVarArray(PairingNum, 0, 1);
        IloRange[] rng = new IloRange[FlightNum];

        IloNumExpr obj = lpcplex.prod(CostLegnumPairing[0][0], p[0]);
        for(int i=1; i<PairingNum; i++){
            obj = lpcplex.sum(obj, lpcplex.prod(CostLegnumPairing[i][0], p[i]));
        }
        lpcplex.addMinimize(obj);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf = lpcplex.prod(0,p[0]);
            for(int i=0; i< PairingNum; i++){
                for(int j=2; j<2+CostLegnumPairing[i][1]; j++){
                    if(CostLegnumPairing[i][j] == f){
                        coverf = lpcplex.sum(coverf, p[i]);
                        break;
                    }
                }
            }
            rng[f] = lpcplex.addGe( coverf, 1);
        }

        lpcplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        lpcplex.setWarning(null);
        lpcplex.setOut(null);
        lpcplex.solve();

        System.out.println("LpObj = " + lpcplex.getObjValue());
//        System.out.println("LpDualValue = " + Arrays.toString(lpcplex.getDuals(rng)));

//        ReducedcostFlightsdual[0]=lpcplex.getObjValue();
//        System.arraycopy(lpcplex.getDuals(rng), 0, ReducedcostFlightsdual, 1, lpcplex.getDuals(rng).length);

        double[] flightsdual = lpcplex.getDuals(rng);
        lpcplex.end();

        return flightsdual;
    }


    static int[] IP(int[][] CostLegnumPairing, int PairingNum, int FlightNum) throws IloException {
        IloCplex ipcplex = new IloCplex();

        IloIntVar[] p = ipcplex.boolVarArray(PairingNum);
        IloRange[] rng = new IloRange[FlightNum];

        IloIntExpr obj = ipcplex.prod(CostLegnumPairing[0][0], p[0]);
        for(int i=1; i<PairingNum; i++){
            obj = ipcplex.sum(obj, ipcplex.prod(CostLegnumPairing[i][0], p[i]));
        }
        ipcplex.addMinimize(obj);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf = ipcplex.prod(0,p[0]);
            for(int i=0; i< PairingNum; i++){
                for(int j=2; j<2+CostLegnumPairing[i][1]; j++){
                    if(CostLegnumPairing[i][j] == f){
                        coverf = ipcplex.sum(coverf, p[i]);
                        break;
                    }
                }
            }
            rng[f] = ipcplex.addEq( coverf, 1);
        }

        ipcplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        ipcplex.setWarning(null);
        ipcplex.setOut(null);
        ipcplex.solve();

        System.out.println("\nIpObj = " + ipcplex.getObjValue());

        int[] choosep = new int[p.length];
        for(int i=0; i<p.length; i++)
        {
            choosep[i] = (int) ipcplex.getValue(p[i]);
        }
        ipcplex.end();

        return choosep;
    }


    static int maxlegs = 8;
    static int maxworktime = 600;
    static int maxflytime = 480;
    static int minpay = 10000;
    static int workpayrate = 111;
    static int flypayrate = 222;
    static double epson = 1e-6;

    static int NumberOfInitialPairings = 6;

    public static void main(String[] args) throws  IloException,Exception {

        IloCplex timecplex = new IloCplex();
        double t0 = timecplex.getCplexTime();

        System.out.println("Start crew pairing!");

        DataReader flightsdata = new DataReader("src/crewpairing/data/flights.txt");

        int[][] flights = new int[168][5];

        for (int i = 0; i < flights.length; i++) {
            for (int j = 0; j < flights[0].length; j++){
                flights[i][j] = flightsdata.next();
            }
        }

//        for(int i=0;i<168;i++){
//            System.out.println(Arrays.toString(flights[i]));
//        }

        DataReader airportsdata = new DataReader("src/crewpairing/data/airports.txt");
        int[][] airports = new int[5][3];

        for (int i = 0; i < airports.length; i++) {
            for (int j = 0; j < airports[0].length; j++){
                airports[i][j] = airportsdata.next();
            }
        }

//        for(int i=0;i<5;i++){
//            System.out.println(Arrays.toString(airports[i]));
//        }

        int latestArrivalTime = 0;
        int[] flightsdeps = new int[flights.length];
        int[] flightsarrs = new int[flights.length];
        int[] flightsdept = new int[flights.length];
        int[] flightsarrt = new int[flights.length];
        int[] flightsflytime = new int[flights.length];
        int[] flightstsmin = new int[flights.length];
        int[] flightstsmax = new int[flights.length];
        for(int i=0;i<flights.length;i++){
            flightsdeps[i] = flights[i][1];
            flightsarrs[i] = flights[i][2];
            flightsdept[i] = flights[i][3];
            flightsarrt[i] = flights[i][4];
            flightsflytime[i] = flights[i][4] -flights[i][3];
            flightstsmin[i] = airports[flights[i][1]][1];
            flightstsmax[i] = airports[flights[i][1]][2];
            if(flights[i][4]>latestArrivalTime){
                latestArrivalTime = flights[i][4];
            }
        }

        int[][] CostLegnumPairing = new int[10000][10];
        int PairingNum = 0;


        double t1 = timecplex.getCplexTime();

        System.out.println("\nStart initialization process:"+"\nNumber of initial pairings for each flight = "+NumberOfInitialPairings);
        for(int i=0;i<flights.length;i++) {
            if(i%10==0){
                System.out.println("initialization process: "+(i+1)+"/"+flights.length);
            }
            int[][] OneflightCostLegnumPairing = onepairing(flightsdeps, flightsarrs, flightsdept, flightsarrt,
                                                            flightsflytime, flightstsmin, flightstsmax, latestArrivalTime, i);
            for(int n=0; n<NumberOfInitialPairings; n++){
                CostLegnumPairing[NumberOfInitialPairings*i+n] = OneflightCostLegnumPairing[n];
            }
        }
        System.out.println("finsh initialization: "+ flights.length+ "/"+flights.length);
        System.out.println("initialization time= "+ (timecplex.getCplexTime()-t1));
//        for(int i=0; i<flights.length*NumberOfInitialPairings; i++){
//            System.out.println(Arrays.toString(CostLegnumPairing[i]));
//        }

        PairingNum = flights.length*NumberOfInitialPairings ;


        int LpLoop = 0;
        for( ; ; ){
            LpLoop += 1;
            System.out.println("\nLpLoop " + LpLoop + ":");
            double[] flightsdual = LP(CostLegnumPairing, PairingNum, flights.length);

            double[] ReducedcostCostActualPairing = onebestpairing(flightsdeps, flightsarrs, flightsdept, flightsarrt,
                    flightsflytime, flightstsmin, flightstsmax, latestArrivalTime, flightsdual);
            System.out.println("Reducedcost = " + ReducedcostCostActualPairing[0]);
            if ( ReducedcostCostActualPairing[0] <= -epson ){
                for(int i = 0; i<10; i++){
                    CostLegnumPairing[PairingNum][i] = (int) ReducedcostCostActualPairing[i+1];
                }
                PairingNum += 1;
            }else{
                break;
            }
        }

        int[] choosep = IP(CostLegnumPairing, PairingNum, flights.length);

        System.out.println("Solution = ");
        int choosepnum = 0;
        for(int i=0; i<PairingNum; i++){
            if( choosep[i] == 1){
                choosepnum += 1;
                for(int j=0; j< CostLegnumPairing[i][1]-1;j++){
                    System.out.print(CostLegnumPairing[i][2+j] + "->");
                }
                System.out.println(CostLegnumPairing[i][CostLegnumPairing[i][1]+1]);
            }
        }
        System.out.println("IP covering uses: "+ choosepnum + " out of " + PairingNum + " pairings");

        int[] flightcovernum = new int[flights.length];
        int chongfu = 0;
        for(int f=0; f<flights.length; f++){
            flightcovernum[f] = 0;
            for(int i=0; i< PairingNum; i++){
                for(int j=2; j<2+CostLegnumPairing[i][1]; j++){
                    if(choosep[i] == 1 && CostLegnumPairing[i][j] == f){
                        flightcovernum[f] += 1;
                        break;
                    }
                }
            }

            if(flightcovernum[f]>1){
                chongfu += 1;
            }
        }
        System.out.println("Find " + chongfu + " flight(s) is/are covered more than once");

        System.out.println("\ntotal time = "+ (timecplex.getCplexTime()-t0));
    }

}
