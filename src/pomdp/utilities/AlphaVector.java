/*
 * Created on May 6, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package pomdp.utilities;

import java.io.Serializable;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import pomdp.environments.POMDP;

/**
 * @author Guy Shani
 *
 * This class represents an Alpha vector, assigning a value for each state.
 */


public abstract class AlphaVector implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	protected BeliefState m_bsWitness;
	protected int m_cStates;
	protected int m_cActions;
	protected int m_cObservations;
	protected int m_iAction;
	protected double m_dMaxValue;
	protected double m_dAvgValue;
	protected AlphaVector[][] m_aCachedG;
	protected double m_dOffset;
	protected POMDP m_pPOMDP;
	public boolean m_bMaintainWitness = true;
	protected static boolean s_bAllowCaching = false;

	//private boolean m_bDominated;
	
	//这些抽象方法还要在继续改写
	public abstract double valueAt(int iState);
	public abstract void setValue( int iState, double dValue );
	public abstract int getNonZeroEntriesCount();
	public abstract Iterator<Entry<Integer,Double>> getNonZeroEntries();
	public abstract AlphaVector newAlphaVector();
	public abstract void finalizeValues();
	public abstract void accumulate( AlphaVector av );
	
	public AlphaVector( BeliefState bsWitness, int iAction, POMDP pomdp )
	{
		m_bsWitness = bsWitness;
		m_pPOMDP = pomdp;
		
		m_cStates = m_pPOMDP.getStateCount();
		m_cActions = m_pPOMDP.getActionCount();
		m_cObservations = m_pPOMDP.getObservationCount();
		m_iAction = iAction;
		
		m_aCachedG = new AlphaVector[m_cActions][m_cObservations];
	}
	
	public boolean dominates(AlphaVector avOther){
		if( getMaxValue() < avOther.getMaxValue() )//最大值小
			return false;
		
		if( getAvgValue() < avOther.getAvgValue() )//平均值小
			return false;
		
		for( int iState : m_pPOMDP.getValidStates() ){//某一个状态的值小
			double dValue = valueAt( iState );
			double dOtherValue = avOther.valueAt( iState );
			if( dOtherValue > dValue )
				return false;
		}
		return true;
	}
	
	public double getMaxValue(){
		
		return m_dMaxValue;
	}
	
	public double getAvgValue(){
		return m_dAvgValue;
	}
	
	public BeliefState getWitness(){
		return m_bsWitness;
	}
	
	public int getAction(){
		return m_iAction;
	}
	
	public void setAction( int iAction ){
		m_iAction = iAction;
	}
	
	public void setWitness( BeliefState bsWitness ){
		if( m_bMaintainWitness )
			m_bsWitness = bsWitness;
	}
	
	public void setAllValues( double dValue ) {
		for( int iState : m_pPOMDP.getValidStates() ){
			setValue( iState, dValue );
		}		
	}
	
	public void release(){
		int iAction = 0, iObservation = 0;
		if( m_aCachedG != null ){
			for( iAction = 0 ; iAction < m_aCachedG.length ; iAction++ ){
				if( m_aCachedG[iAction] != null ){
					for( iObservation = 0 ; iObservation< m_cObservations ; iObservation++ ){
						if( m_aCachedG[iAction][iObservation] != null )
							m_aCachedG[iAction][iObservation].release();
					}
				}
			}
		}
	}
	
	public AlphaVector addReward( int iAction ){
		AlphaVector avResult = newAlphaVector();
		double dValue = 0.0;
		for( int iState : m_pPOMDP.getValidStates() ){
			dValue = valueAt( iState ) * m_pPOMDP.getDiscountFactor() + m_pPOMDP.R( iState, iAction );
			avResult.setValue( iState, dValue );
		}
		avResult.finalizeValues();
		return avResult;
	}
	
	@SuppressWarnings({ "rawtypes", "unchecked" })
	public AlphaVector copy(){
		AlphaVector avCopy = newAlphaVector();
			
		int iState = 0;
		double dValue = 0.0;
		
		Iterator<Entry<Integer, Double>> itNonZero = getNonZeroEntries();
		if( itNonZero != null ){

			Pair<Integer,Double> p = null;
			Map.Entry e = null;
			Object oElement = null;
			
			while( itNonZero.hasNext() ){
				oElement = itNonZero.next();
				if( oElement instanceof Pair ){
					p = (Pair) oElement;
					iState = ((Number)p.m_first).intValue();
					dValue = ((Number)p.m_second).doubleValue();
					avCopy.setValue( iState, dValue );
				}
				else if( oElement instanceof Map.Entry ){
					e = (Map.Entry) oElement;
					iState = ((Number)e.getKey()).intValue();
					dValue = ((Number)e.getValue()).doubleValue();
					avCopy.setValue( iState, dValue );
				}
			}
		}
		else{
			for( int iValidState : m_pPOMDP.getValidStates() ){
				dValue = valueAt( iValidState );
				avCopy.setValue( iValidState, dValue );
			}
		}
		
		return avCopy;
	}
	
	public AlphaVector G( int iAction, int iObservation ){
		if( s_bAllowCaching && ( m_aCachedG[iAction][iObservation] != null ) )
			return m_aCachedG[iAction][iObservation];
		
		AlphaVector avResult = computeG( iAction, iObservation );
		if( s_bAllowCaching )
			m_aCachedG[iAction][iObservation] = avResult;
		return avResult;
	}
	
	protected synchronized AlphaVector computeG( int iAction, int iObservation ){
		int iStartState = 0, iEndState = 0;
		double dObservation = 0.0, dTr = 0.0, dValue = 0.0, dSum = 0.0;

		AlphaVector avResult = newAlphaVector();
		avResult.setAction( iAction );

		Iterator<Entry<Integer,Double>> itNonZeroEntries = null;//第一个参数是转移后的状态，第二个参数是概率
		Entry<Integer,Double> eValue = null;
		
		for( iStartState = 0 ; iStartState < m_cStates ; iStartState++ ){
			dSum = 0.0;
			itNonZeroEntries = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );//状态转移
			
			while( itNonZeroEntries.hasNext() ){
				eValue = itNonZeroEntries.next();
				iEndState = eValue.getKey();
				dValue = valueAt( iEndState );
				dTr = eValue.getValue();//概率
				if( dValue != 0 ){
					dObservation = m_pPOMDP.O( iAction, iEndState, iObservation );
					dSum += dObservation * dTr * dValue;
				}
			}
			
			if( dSum != 0 ){
				avResult.setValue( iStartState, dSum );//每一个状态的值进行更新
			}
		}
		avResult.finalizeValues();
		return avResult;
	}
	public double dotProduct(BeliefState bs)//信念点在向量的投影
	{
		if(bs == null)
			return 0.0;
		
		int cBeliefNonZeroEntries = bs.getNonZeroEntriesCount();
		int cAlphaNonZeroEntries = getNonZeroEntriesCount();
		double dSum = 0.0, dProb = 0.0, dValue = 0.0;
		int iState = 0;
		Entry<Integer,Double> e = null;
		
		Iterator<Entry<Integer,Double>> iter;
		if( cBeliefNonZeroEntries < cAlphaNonZeroEntries ){
			iter = bs.getNonZeroEntries().iterator();
			if( iter != null ){
				while( iter.hasNext() ){
					e =  iter.next();
					iState = e.getKey().intValue();//状态
					dProb = e.getValue().doubleValue();//在该状态的概率
					dValue = valueAt( iState );//在该状态的值
					dSum += dValue * dProb;
				}
			}
		}
		else{
			iter = getNonZeroEntries();
			if( iter != null ){
				while( iter.hasNext() ){
					e = iter.next();
					iState = ((Integer) e.getKey()).intValue();
					dValue = ((Double) e.getValue()).doubleValue();
					dProb = bs.valueAt( iState );
					dSum += dValue * dProb;
				}
			}			
		}
		
		if( iter == null ){
			for( iState = 0 ; iState < m_cStates ; ++iState ){
				dProb = bs.valueAt( iState );
				dValue = valueAt( iState );
				dSum += dValue * dProb;
			}
		}
		return dSum;
	}
	
	public void parseValuesXML( Element eFunction ){
		int iStateItem = 0, iState = 0;
		double dValue = 0;
		Element eState = null;
		NodeList nlStates = eFunction.getChildNodes();
		for( iStateItem = 0 ; iStateItem < nlStates.getLength() ; iStateItem++ ){
			eState = (Element)nlStates.item( iStateItem );
			dValue = Double.parseDouble( eState.getAttribute( "Value" ) );
			iState = Integer.parseInt( eState.getAttribute( "Id" ) );
			setValue( iState, dValue );
		}
		finalizeValues();
	}
	
	public static AlphaVector parseDOM( Element eVector, POMDP pomdp ) throws Exception{
		AlphaVector avNew = null;
		avNew = new TabularAlphaVector( null, 0.0, pomdp );			
		
		avNew.setAction( Integer.parseInt( eVector.getAttribute( "Action" ) ) );
		avNew.parseValuesXML( eVector );

		return avNew;
	}
	public Element getDOM( Document docValueFunction ) throws Exception{
		Element eVector = docValueFunction.createElement( "AlphaVector" );
		Element eState = null;
		
		Iterator<Entry<Integer,Double>> itAlphaVectorNonZero = getNonZeroEntries();
		Entry<Integer,Double> e = null;
		int iState = 0, cNonZeroStates = 0;
		double dValue = 0.0;
		
		if( itAlphaVectorNonZero != null ){
			while( itAlphaVectorNonZero.hasNext() ){
				e = itAlphaVectorNonZero.next();
				iState = e.getKey();
				dValue = e.getValue();
				eState = docValueFunction.createElement( "State" );
				eState.setAttribute( "Id", iState + "" );
				eState.setAttribute( "Value", dValue + "" );
				
				eVector.appendChild( eState );
			}
			cNonZeroStates = getNonZeroEntriesCount();
		}
		else{
			for( iState = 0 ; iState < m_cStates ; iState++ ){
				dValue = valueAt( iState );
				if( dValue != 0.0 ){
					eState = docValueFunction.createElement( "State" );
					eState.setAttribute( "Id", iState + "" );
					eState.setAttribute( "Value", dValue + "" );
					
					eVector.appendChild( eState );	
					
					cNonZeroStates++;
				}
			}
		}
				
		//eVector.setAttribute( "Id", m_iID + "" );
		eVector.setAttribute( "EntriesCount", cNonZeroStates + "" );
		eVector.setAttribute( "Action", m_iAction + "" );
		eVector.setAttribute( "Type", "Flat" );

		return eVector;
	}
}