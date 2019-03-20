package pomdp.algorithms;
/*
 * Created on May 6, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

/**
 * @author shanigu
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

import java.util.Iterator;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.Logger;
import pomdp.utilities.RandomGenerator;
import pomdp.valuefunction.LinearValueFunctionApproximation;
import pomdp.valuefunction.MDPValueFunction;

public abstract class ValueIteration extends PolicyStrategy{
	protected MDPValueFunction m_vfMDP;
	protected LinearValueFunctionApproximation m_vValueFunction;
	protected POMDP m_pPOMDP;
	protected int m_cStates;
	protected int m_cActions;
	protected int m_cObservations;
	protected double m_dGamma;
	protected AlphaVector m_avMaxValues;
	protected final static double MIN_INF = Double.NEGATIVE_INFINITY;
	protected final static double MAX_INF = Double.POSITIVE_INFINITY;
	protected boolean m_bConverged;
	//�� 0.001
	protected double m_dEpsilon = 0.001;
	protected long m_cElapsedExecutionTime;
	protected long m_cCPUExecutionTime;
	protected long m_cDotProducts;
	protected int m_cValueFunctionChanges;
	protected boolean m_bTerminate;
	
	protected long m_cTimeInBackup;
	protected long m_cTimeInHV;
	protected long m_cTimeInV;
	protected long m_cAlphaVectorNodes; 
	
	protected static int g_cTrials = 500;
	protected static int g_cStepsPerTrial = 100;
	
	//û���õ��ı�����û�еط���������
	protected static String m_sBlindPolicyValueFunctionFileName = null;
	
	protected RandomGenerator m_rndGenerator;	
	

	public ValueIteration( POMDP pomdp ){
		m_pPOMDP = pomdp;
		m_cStates = m_pPOMDP.getStateCount();
		m_cActions = m_pPOMDP.getActionCount();
		m_cObservations = m_pPOMDP.getObservationCount();
		m_dGamma = m_pPOMDP.getDiscountFactor();
		m_avMaxValues = null;
		m_bConverged = false;
		m_cElapsedExecutionTime = 0;
		m_cCPUExecutionTime = 0;
		//m_dFilteredADR = 0.0;
		m_cDotProducts = 0;
		
		m_cTimeInBackup = 0;
		m_cTimeInHV = 0;
		m_cTimeInV = 0;
		m_cAlphaVectorNodes = 0;
		
		m_bTerminate = false;
		
		//�������������
		m_rndGenerator = new RandomGenerator( "ValueIteration" );
		
		//m_dEpsilon: �� 0.001
		m_vValueFunction = new LinearValueFunctionApproximation( m_dEpsilon, true );
		m_vfMDP = pomdp.getMDPValueFunction();

		//computeStepsPerTrial();
		initValueFunctionUsingBlindPolicy();

	}
	
	// 2018-4-21
	public abstract void newIteration(POMDP pomdp);
	
	private double findMaxAlphas( int iAction, BeliefState bs, LinearValueFunctionApproximation vValueFunction, AlphaVector[] aNext ) {
		AlphaVector avAlpha = null;
		int iObservation = 0;
		double dSumValues = 0.0, dValue = 0, dProb = 0.0, dSumProbs = 0.0;
		BeliefState bsSuccessor = null;
		
		boolean bCache = m_pPOMDP.getBeliefStateFactory().isCachingBeliefStates();
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){//����ÿһ���۲죬����������ֵ�������������ر�ֵ
			dProb = bs.probabilityOGivenA( iAction, iObservation );//����ִ�ж���a�õ��۲�o�ĸ���
			dSumProbs += dProb;
			if( dProb > 0.0 ){
				bsSuccessor = bs.nextBeliefState( iAction, iObservation );//����b��a��o�ĺ�������
				avAlpha = vValueFunction.getMaxAlpha( bsSuccessor );//����������ֵ������ֵ���ʱ����Ӧ������
				dValue = avAlpha.dotProduct( bsSuccessor );//����������ֵ�����ж�Ӧ�����ֵ
				dSumValues += dValue * dProb;//��Ȩ���
			}
			else{
				avAlpha = vValueFunction.getLast();
			}
			aNext[iObservation] = avAlpha;//ÿһ��0����һ���Ժ��Ӧ���������
			
		}
		
		//ʽF10
		//�ͱ�׼backup�е�ʽ�Ӳ�һ��
		dSumValues /= dSumProbs; //in case due to rounding there is an error and probs do not exactly sum to 1
		dSumValues *= m_pPOMDP.getDiscountFactor();
		dSumValues += m_pPOMDP.immediateReward( bs, iAction ); //���������ر�
		
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bCache );
		
		return dSumValues;
	}
	
	private AlphaVector G( int iAction, LinearValueFunctionApproximation vValueFunction, AlphaVector[] aNext) {
		AlphaVector avAlpha = null, avG = null, avSum = null, avResult = null;
		int iObservation = 0;
		
		//����o���Ѧ����
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			avAlpha = aNext[iObservation];
			avG = avAlpha.G( iAction, iObservation );
				//Logger.getInstance().logln( iObservation + ") " + avAlpha + ", " + avG );
			if( avSum == null )
				avSum = avG.copy();
			else
				avSum.accumulate( avG );
		}
		
		//���������ر�
		avResult = avSum.addReward( iAction );
		avResult.setAction( iAction );

		avSum.release();
		return avResult;
	}
	
	public AlphaVector backup( BeliefState bs ){
		return backup( bs, m_vValueFunction );
	}
	
	public AlphaVector backup( BeliefState bs, LinearValueFunctionApproximation vValueFunction ){
		
		return  backupTauBased( bs, vValueFunction);
	}
	
	protected AlphaVector backupTauBased( BeliefState bs, LinearValueFunctionApproximation vValueFunction){
		double dValue = 0.0, dMaxValue = Double.NEGATIVE_INFINITY;
		int iAction = 0, iMaxAction = -1;
		AlphaVector[] aNext = null, aBest = null;
	
		Vector<AlphaVector[]> vWinners = new Vector<AlphaVector[]>();//������Ϊ��ͬ�����ж��������Ū��vector
		Vector<Integer> vWinnersActions = new Vector<Integer>();//����action
	
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){//ÿ��action���Ŧ�
			aNext = new AlphaVector[m_cObservations];//aNext�����b��a��V���ҵ�ÿ��o��Ӧ������
				//����ֵΪF10������b��a�̶�ʱ������һ�������õ������value����ÿ��o����һ�����Ӧ������������
			dValue = findMaxAlphas( iAction, bs, vValueFunction, aNext );
			
			if( dValue > dMaxValue ){
				dMaxValue = dValue;
				vWinners.clear();
				vWinnersActions.clear();
			}
			if( dValue == dMaxValue ){
				aBest = aNext;//aBest�ŵ������action��Ӧ��aNext
		                      //b��a��o�̶�ʱ�����action�£�ÿ��o����һ�����Ӧ������
				iMaxAction = iAction;
				vWinners.add( aBest );//��¼ȡֵ���Ķ�������Ӧ������
				vWinnersActions.add( iMaxAction );//��¼ȡֵ���Ķ���
			}
		}
		
		//����ж�����action�����ȡһ��
		int idx = m_rndGenerator.nextInt( vWinners.size() );
		aBest = vWinners.elementAt( idx );
		iMaxAction = vWinnersActions.elementAt( idx );
		//��������º�Ħ�
		AlphaVector avMax = G( iMaxAction, vValueFunction, aBest );
		avMax.setWitness( bs );
		
		return avMax;
	}
	protected double getMaxMinR(){
		return m_pPOMDP.getMaxMinR();
	}
	
	//ʹ��blind���ԣ���ʼ��ֵ����V LinearValueFunctionApproximation
	protected void initValueFunctionUsingBlindPolicy(){
		m_vValueFunction.clear();
		
		int iAction = 0, iState = 0, iEndState = 0, iIteration = 0;
		double dValue = 0.0, dNewValue = 0.0, dReward = 0.0, dDiff = 0.0, dMaxDiff = 0.0, dTr = 0.0, dSum = 0.0;
		AlphaVector av = null;
		AlphaVector avNext = null;
		double dMaxResidual = MAX_INF;
		Entry<Integer,Double> entry = null;
		Iterator<Entry<Integer,Double>> itNonZero = null;
		LinearValueFunctionApproximation vMin = new LinearValueFunctionApproximation( m_dEpsilon, false );
		//��ʼ��Ϊ��Сֵ
		initValueFunctionToMin( vMin );
		
		//���״̬���ʾ�һ�µ�b
		BeliefState bsUniform = m_pPOMDP.getBeliefStateFactory().getUniformBeliefState();
		
		if( m_sBlindPolicyValueFunctionFileName != null ){
			try{
				m_vValueFunction.load( m_sBlindPolicyValueFunctionFileName, m_pPOMDP );
				Logger.getInstance().logln( "Blind policy loaded successfully" );
				return;
			}
			catch( Exception e ){
				Logger.getInstance().logln( "Could not load blind policy - " + e );
			}
		}
		Logger.getInstance().logln( "Begin blind policy computation  " + m_cActions + " actions" );
		
		//MDPֵ����������������
		//����action
		//����F3����ÿ��state��ֵ��ȡ���ģ�����¼��Ӧ��action
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			//����F2��MDP��ĳaction�£�������state����ֵ����������
			av = vMin.elementAt( 0 ); //ȡ����һ��������ʵ��ֻ��һ����
			//av = m_pPOMDP.newAlphaVector();
			//Logger.getInstance().logln( "Initial " + av );
			iIteration = 0;
			dMaxResidual = MAX_INF;
			//����������������������ǰ�����value�Ĳ�ֵ��<=0.1
			while( dMaxResidual > 0.1 ){
				avNext = av.newAlphaVector();
				dMaxDiff = 0.0;
				//����state
				//MDP��������״̬����һ��ֵ������Ȼ��ѵ�����ֵ�������¦� avNext
				//state 1
				for( iState = 0 ; iState < m_cStates ; iState++ ){
					//-------����F1��S����MDP��һ��ֵ����
					dSum = 0.0;
					//��������state action��ʼ��ת��
					//state2
					itNonZero = m_pPOMDP.getNonZeroTransitions( iState, iAction );
					while( itNonZero.hasNext() ){
						entry = itNonZero.next();
						iEndState = entry.getKey().intValue();
						dTr = entry.getValue().doubleValue();//ת���ĸ���ֵ
						dValue = av.valueAt( iEndState );//�������״̬�ĵ�value
						dSum += dTr * dValue;//
					}
					dReward = m_pPOMDP.R( iState, iAction );
					dNewValue = dReward + dSum * m_dGamma;
					//-------
					//��MDP��һ��state��һ��������ֵ�����µĦ�
					avNext.setValue( iState, dNewValue );
					
					//����һ�������󣬲�ֵ����state
					dDiff = Math.abs( dNewValue - av.valueAt( iState ) );
					if( dDiff > dMaxDiff ){
						dMaxDiff = dDiff;
					}
				}
				
				dMaxResidual = dMaxDiff;
				avNext.finalizeValues();
				//�¦�����Ϧ�
				av = avNext;
				
				iIteration++;
				
				if( iIteration % 10 == 0 )
					Logger.getInstance().log( "." );
			}
						
			av.setWitness( bsUniform );
			av.setAction( iAction );//���ö�Ӧ��action
			m_vValueFunction.addPrunePointwiseDominated( av );//�����ȡ��ֵ���Ħ�
			Logger.getInstance().logln( "Done action " + iAction +
					" after " + iIteration + " iterations |V| = " + m_vValueFunction.size() );
		}		
		Logger.getInstance().logln( "Done blind policy" );
		
		if( m_sBlindPolicyValueFunctionFileName != null ){
			try{
				m_vValueFunction.save( m_sBlindPolicyValueFunctionFileName );
				Logger.getInstance().logln( "Blind policy saved successfully" );
				return;
				
			}
			catch( Exception e ){
				Logger.getInstance().logln( "Could not save blind policy - " + e );
			}
		}
		
	}
	
	protected void initValueFunctionToMin( LinearValueFunctionApproximation vValueFunction ){
		Logger.getInstance().logln( "Init value function to min" );
		double dMinValue = getMaxMinR();
		double dDefaultValue = dMinValue / ( 1 - m_dGamma );

 		BeliefState bsUniform = m_pPOMDP.getBeliefStateFactory().getUniformBeliefState();
		Logger.getInstance().logln( "Min R value = " + dMinValue + " init value = " + dDefaultValue );
	
		AlphaVector avMin = null;
		avMin = m_pPOMDP.newAlphaVector();
		avMin.setAllValues( dDefaultValue );
		avMin.finalizeValues();
		avMin.setWitness( bsUniform );
		vValueFunction.add( avMin );
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	protected Iterator backwardIterator( Vector vElements ){
		Vector vBackward = new Vector();
		int iElement = 0, cElements = vElements.size();
		for( iElement = cElements - 1 ; iElement >= 0 ; iElement-- ){
			vBackward.add( vElements.get( iElement ) );
		}
		return vBackward.iterator();
	}

	
	
}
