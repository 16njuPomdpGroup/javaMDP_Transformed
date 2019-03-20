package pomdp.utilities;

import java.util.Iterator;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.utilities.datastructures.StaticMap;

public class TabularAlphaVector extends AlphaVector{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private double[] m_aValues;//保存每一个状态对应的值
	private StaticMap m_mValues;
	
	
	public TabularAlphaVector( BeliefState bsWitness, double dDefaultValue, POMDP pomdp ){
		this( bsWitness, 0, pomdp );
		int iState = 0;
		for( iState = 0 ; iState < m_cStates ; iState++ ){
			m_aValues[iState] = dDefaultValue;
		}
		//m_dOffset = 0.0;
		m_mValues = null;

	}
	
	public TabularAlphaVector( BeliefState bsWitness, int iAction, POMDP pomdp ){
		super( bsWitness, iAction, pomdp );
		m_aValues = new double[m_cStates];
		m_mValues = null;
	}
	
	public void accumulate( AlphaVector av ){
		int iState = 0;
		double dValue = 0.0, dLocalValue = 0.0;
		Iterator<Entry<Integer, Double>> itNonZero = av.getNonZeroEntries();
		
		if( itNonZero != null ){
			Pair<Integer,Double> p = null;
			while( itNonZero.hasNext() ){
				p = (Pair<Integer,Double>)itNonZero.next();
				iState = p.m_first.intValue();
				dValue = p.m_second.doubleValue();
				dLocalValue = valueAt( iState );
				setValue( iState, dValue + dLocalValue );
			}
		}
		else{
			for( iState = 0 ; iState < m_cStates ; iState++ ){
				setValue( iState, valueAt( iState ) + av.valueAt( iState ) );
			}
		}
	}
	@Override
	public void finalizeValues() {//不懂
		m_mValues = new StaticMap( m_aValues, 0.001 );	
		m_aValues = null;
	}

	@Override
	public Iterator<Entry<Integer, Double>> getNonZeroEntries() {
		if( m_mValues == null )
			finalizeValues();
		return m_mValues.iterator();
	}

	
	public int getNonZeroEntriesCount() {
		if( m_mValues == null )
			return m_cStates;
		return m_mValues.size();
	}

	@Override
	public AlphaVector newAlphaVector() {
		AlphaVector avResult = new TabularAlphaVector( null, 0, m_pPOMDP );
		return avResult;
	}

	@Override
	public void setValue(int iState, double dValue) {
		if( dValue > m_dMaxValue )
			m_dMaxValue = dValue;
		m_dAvgValue += dValue / m_cStates;
		m_aValues[iState] = dValue;
	}

	@Override
	public double valueAt(int iState) {
		if( m_aValues != null )
			return m_aValues[iState];
		else			
			return m_mValues.get( iState );
	}

}
