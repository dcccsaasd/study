package examples;/* --------------------------------------------------------------------------
 * File: InOut1.java
 * Version 12.9.0  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2019. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 * 
 * Problem Description
 * -------------------
 * (1)P0,P1,P2代表三种产品
 * (2)x1,x3,x5对应公司自己生产的三种产品的数量
 * (3)x2,x4,x6对应公司从外部购买的三种产品的数量
 * (4)生产需要原料，所以只针对x1,x3,x5有原料相关约束
 * (5)x1+x2=公司对产品P0的需求数量，以此类推。。。
 * A company has to produce 3 products, using 2 resources.
 * Each resource has a limited capacity.
 * Each product consumes a given number of machines.
 * Each product has a production cost (the inside cost).
 * Both products can also be purchased outside the company at a given
 * cost (the outside cost).
 * 
 * Minimize the total cost so that the company exactly meets the
 * demand.
 */

import ilog.concert.*;
import ilog.cplex.*;

public class InOut1 {
   static int _nbProds = 3;
   static int _nbResources = 2;
   static double[][] _consumption = {{0.5, 0.4, 0.3},
                                     {0.2, 0.4, 0.6}};
   static double[] _demand = {100.0, 200.0, 300.0};
   static double[] _capacity = {20.0, 40.0};
   static double[] _insideCost = {0.6, 0.8, 0.3};
   static double[] _outsideCost = {0.8, 0.9, 0.4};
   //static double[] _outsideCost = {0.8, .09, 0.4};这里的.09应该是错了(程序中默认.09=0.09),更正为0.9
   static void displayResults(IloCplex cplex,
                              IloNumVar[] inside,
                              IloNumVar[] outside) throws IloException {
      System.out.println("cost: " + cplex.getObjValue());
      //System.out.println(".09是"+_outsideCost[1]);
      for(int p = 0; p < _nbProds; p++) {
         System.out.println("P" + p);
         System.out.println("inside:  " + cplex.getValue(inside[p]));
         System.out.println("outside: " + cplex.getValue(outside[p]));
      }
   }
   
   public static void main( String[] args ) {
      try {
         IloCplex cplex = new IloCplex();
       
         IloNumVar[]  inside = new IloNumVar[_nbProds];
         IloNumVar[] outside = new IloNumVar[_nbProds];
       
         IloObjective obj = cplex.addMinimize();
       
         // Must meet demand for each product
         for(int p = 0; p < _nbProds; p++) {

            //约束的上下界，循环一次添加一行约束
            IloRange demRange = cplex.addRange(_demand[p], _demand[p]);

            //numVar()添加变量，一次循环里有inside[p]和outside[p]，说明循环一次添加两列变量
            //与LPex1的cplex.numVar()相比参数少了对变量名称的指定，这里是默认依次为x1,x2,x3...
            inside[p] = cplex.numVar(cplex.column(obj, _insideCost[p]).and(
                                     cplex.column(demRange, 1.)),
                                     0., Double.MAX_VALUE);
            
            outside[p] = cplex.numVar(cplex.column(obj, _outsideCost[p]).and(
                                      cplex.column(demRange, 1.)),
                                      0., Double.MAX_VALUE);
         }
       
         // Must respect capacity constraint for each resource
       
         for(int r = 0; r < _nbResources; r++)
            cplex.addLe(cplex.scalProd(_consumption[r], inside), _capacity[r]);
         cplex.solve();
         cplex.exportModel("Inout1.lp");
       
         if ( !cplex.getStatus().equals(IloCplex.Status.Optimal) ) {
            System.out.println("No optimal solution found");
            return;
         }
         System.out.println("Solution status: " + cplex.getStatus());       
         displayResults(cplex, inside, outside);
         System.out.println("----------------------------------------");
         cplex.end();
      }
      catch (IloException exc) {
         System.err.println("Concert exception '" + exc + "' caught");
      }
   }
}
  
/*
cost: 372
P0
inside:  40
outside: 60
P1
inside:  0
outside: 200
P2
inside:  0
outside: 300
*/
