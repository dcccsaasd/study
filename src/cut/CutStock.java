/* --------------------------------------------------------------------------
 * File: CutStock.java
 * Version 12.6.3  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2015. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 */

package cut;

import ilog.concert.*;
import ilog.cplex.*;
import java.io.*;

class CutStock {
   static double RC_EPS = 1.0e-6;
   
   // Data of the problem
   static double   _rollWidth;//ľ�ĳ���
   static double[] _size;//ľ������ߴ�
   static double[] _amount;//������
  //�������ܣ���dat�ļ��ж�ȡ����
   static void readData(String fileName)// ��ȡ����
                         throws IOException,
                                InputDataReader.InputDataReaderException {
      //��������reader
      InputDataReader reader = new InputDataReader(fileName);
      //��ȡ������������    
      _rollWidth = reader.readDouble();
      System.out.println(_rollWidth);
      _size      = reader.readDoubleArray();//�ɶ����readDoubleArray()������֪��һ�εĶ�ȡ�ӡ�[����ʼ���ԡ�]������
      _amount    = reader.readDoubleArray();
      for (int i = 0; i <_size.length; i++) {
    	  System.out.print(_size[i]+" ");
	}
      System.out.println();//�������
      for (int i = 0; i <_amount.length; i++) {
    	  System.out.print(_amount[i]+" ");
	}
      System.out.println();
   }
   //��������˶��ٸ�17Ӣ�ߵ�ľ���Լ�ÿ���з����˶���ľ��
   static void report1(IloCplex cutSolver, IloNumVarArray Cut, IloRange[] Fill) 
                         throws IloException {
      System.out.println();
      System.out.println("Using " + cutSolver.getObjValue() + " rolls");//��������˶��ٸ�17Ӣ�ߵ�ľ��
    
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) {
         System.out.println("  Cut" + j + " = " +
                            cutSolver.getValue(Cut.getElement(j)));//���ÿ���з����˶���ľ��
      }
      System.out.println();
      
      for (int i = 0; i < Fill.length; i++) 
         System.out.println("  Fill" + i + " = " + cutSolver.getDual(Fill[i]));//���ÿ��ľ��������з������Ը���
      System.out.println();
   }
 //���Ӱ�Ӽ۸��Լ��µ��з�
   static void report2(IloCplex patSolver, IloNumVar[] Use)
                         throws IloException {
      System.out.println();
      System.out.println("Reduced cost is " + patSolver.getObjValue());//���Ӱ�Ӽ۸�
      
      System.out.println();
      if (patSolver.getObjValue() <= -RC_EPS) {
         for (int i = 0; i < Use.length; i++) 
            System.out.println("  Use" + i + " = "
                               + patSolver.getValue(Use[i]));//����µ��з�
         System.out.println();
      }
   }
 ////��������з�����ľ�����Լ�ÿ���з�����ľ����
   static void report3(IloCplex cutSolver, IloNumVarArray Cut) 
                         throws IloException {
      System.out.println();
      System.out.println("Best integer solution uses " + 
                         cutSolver.getObjValue() + " rolls");//��������з�����ľ����
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) 
         System.out.println("  Cut" + j + " = " + 
                            cutSolver.getValue(Cut.getElement(j)));//���ÿ���з�����ľ����
   }
   //ͳ�Ʊ���ֵ���������
   static class IloNumVarArray {
      int _num           = 0;
      IloNumVar[] _array = new IloNumVar[32];//��ģ����ͨ����������ΪIloNumVar�ı�������Ϊ��IloNumVar�Ķ���,cplex��ذ�
    //���鲻�������ӳ���������
      void add(IloNumVar ivar) {
         if ( _num >= _array.length ) {
            IloNumVar[] array = new IloNumVar[2 * _array.length];
            System.arraycopy(_array, 0, array, 0, _num);
            _array = array;
         }
         _array[_num++] = ivar;
      }

      IloNumVar getElement(int i) { return _array[i]; }
      int       getSize()         { return _num; }
   }

   public static void main(String[] args ) {
      String datafile = "data/cutstock.dat";
      try {
         if (args.length > 0)
            datafile = args[0];
         /// CUTTING-OPTIMIZATION PROBLEM ///
         readData(datafile);//���������ļ�    
         IloCplex cutSolver = new IloCplex();//����Cplex����
         IloObjective RollsUsed = cutSolver.addMinimize();//��Сֵ����
         IloRange[]   Fill = new IloRange[_amount.length];//����cplex������Լ������
         //���Լ��
         for (int f = 0; f < _amount.length; f++ ) {
        	 /*
        	  * ��ÿ�ֳ���ľ����������Լ���������뵽cplex��Լ��������
        	  * Լ������Ϊ��_amount[f]<=Fill[f]
        	  */
            Fill[f] = cutSolver.addRange(_amount[f], Double.MAX_VALUE);//�����˸���Լ�������½�
         }
       //����IloNumVarArray���͵�����
         IloNumVarArray Cut = new IloNumVarArray();
       
         int nWdth = _size.length;
         //System.out.println("nWdth="+Fill[1]);
         for (int j = 0; j < nWdth; j++) {
        	/* 
        	 * 1.����column���ִ���һ���µı��������뵽����RollsUsed��Fill[j]��
        	 * �����ں����е�ϵ���ֱ���1.0��_rollWidth/_size[j]
        	 * 2.numVar�������ֱ�ʾ����һ����ΧΪ( 0.0, Double.MAX_VALUE)�ı���
        	 * �����������뵽column�����еĺ���RollsUsed��Fill[j]��
        	 * 3.��ʽ�ɸ�дΪע�Ͳ������д���
        	 */
            Cut.add(cutSolver.numVar(cutSolver.column(RollsUsed, 1.0).and(
                                     cutSolver.column(Fill[j],
                                                      (int)(_rollWidth/_size[j]))),
                                     0.0, Double.MAX_VALUE));//ÿ��ľ��������з������Ը���
            
//       IloColumn col = cutSolver.column(RollsUsed, 1.0).and(cutSolver.column(Fill[j],
//                       (int)(_rollWidth/_size[j])));
//       Cut.add((cutSolver.numVar(col,0.0, Double.MAX_VALUE)));
         }
         //ѡ�������
         cutSolver.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
         cutSolver.exportModel("xx.lp");
       
        /// �����η�����///
       /// PATTERN-GENERATION PROBLEM ///
         IloCplex patSolver = new IloCplex();
         IloObjective ReducedCost = patSolver.addMinimize();//Ӱ�Ӽ۸�
         /*
          * numVarArray���������������ͱ���ȡֵ��Χ��������������Ԫ�صķ�Χ��ȷ��
          * ����nWdth��ʾ���鳤�ȣ�0��ʾ�½磬double.Max_Value��ʾ�Ͻ�,int��ʾ��������
          */
         IloNumVar[] Use = patSolver.numVarArray(nWdth, 
                                                 0., Double.MAX_VALUE, 
                                                 IloNumVarType.Int);
         /*
          * scalProd��ʾ�������ж�Ӧ��Ԫ����ˣ������˷�
          * addRange��ʾ���patSolver����Լ����Լ����Χ��
          * -Double.MAX_VALUE<patSolver.scalProd(_size, Use)<= _rollWidth
          */
         patSolver.addRange(-Double.MAX_VALUE, 
                            patSolver.scalProd(_size, Use),
                            _rollWidth);

         /// �����ɹ��� ///
       /// COLUMN-GENERATION PROCEDURE ///
         double[] newPatt = new double[nWdth];
         for (;;) {
            //�����ɷ�����
            cutSolver.solve();
            //��������˶��ٸ�17Ӣ�ߵ�ľ���Լ�ÿ���з����˶���ľ��
            report1(cutSolver, Cut, Fill);

          /// �ҵ��Լ�����һ���µĻ����� ///
            //�����ż�����ֵ
            double[] price = cutSolver.getDuals(Fill);
            /*
             * �˱��ʽ����Ӱ�Ӽ�ֵ(reduce cost)
             * ����diff��ʾ����������֮��Ĳ�ֵ
             * setExpr������ʾ��diff��������ֵ����ReduceCost��
             */
            ReducedCost.setExpr(patSolver.diff(1.,
                                               patSolver.scalProd(Use, price)));
          
            patSolver.solve();
          //���Ӱ�Ӽ۸��Լ��µ��з�
            report2 (patSolver, Use);
          
            if ( patSolver.getObjValue() > -RC_EPS )
               break;
          
            newPatt = patSolver.getValues(Use);
          //��������
            IloColumn column = cutSolver.column(RollsUsed, 1.);
            for ( int p = 0; p < newPatt.length; p++ )
               column = column.and(cutSolver.column(Fill[p], newPatt[p]));
          //�����µ��и��
            Cut.add( cutSolver.numVar(column, 0., Double.MAX_VALUE) );
         }
       
         for ( int i = 0; i < Cut.getSize(); i++ ) {
            cutSolver.add(cutSolver.conversion(Cut.getElement(i),
                                               IloNumVarType.Int));
         }
       
         cutSolver.solve();
       //��������з�����ľ�����Լ�ÿ���з�����ľ����
         report3 (cutSolver, Cut);
         System.out.println("Solution status: " + cutSolver.getStatus());       
         cutSolver.end();
         patSolver.end();
      }
      catch ( IloException exc ) {
         System.err.println("Concert exception '" + exc + "' caught");
      }
      catch (IOException exc) {
         System.err.println("Error reading file " + datafile + ": " + exc);
      }
      catch (InputDataReader.InputDataReaderException exc ) {
         System.err.println(exc);
      }
   }
}


/* Example Input file:
17
[3,5,9]
[25,20,15]
*/
