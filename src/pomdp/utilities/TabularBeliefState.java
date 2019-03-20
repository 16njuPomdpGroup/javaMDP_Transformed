package pomdp.utilities;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map.Entry;

import pomdp.utilities.datastructures.StaticMap;

public class TabularBeliefState extends BeliefState {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	protected double[] m_aStateProbabilities;
	protected boolean m_bSparse;
	protected StaticMap m_mNonZeroEntries;
	
	public TabularBeliefState( int cStates, int cActions, int cObservations, int id, boolean bSparse, boolean bCacheBeliefStates, BeliefStateFactory bsFactory ){
		super( cStates, cActions, cObservations, id, bCacheBeliefStates, bsFactory );
		m_bSparse = bSparse;		
		m_aStateProbabilities = new double[m_cStates];
	}
	
	public long size() {
		if( m_bSparse )
			return m_mNonZeroEntries.size();
		else
			return m_cStates;
	}
	public double valueAt( int iState ){
		if( m_aStateProbabilities != null )
			return m_aStateProbabilities[iState];
		else{
			return m_mNonZeroEntries.get( iState );
		}		
	}
	
	public synchronized void setValueAt( int iState, double dValue ){
		if( m_aStateProbabilities != null )
			m_aStateProbabilities[iState] = dValue;
		if( m_mNonZeroEntries != null )
			m_mNonZeroEntries.set( iState, dValue );
		if( dValue != 0.0 ){
			
			if( dValue > m_dMaxBelief ){
				m_dMaxBelief = dValue;
				m_iMaxBeliefState = iState;
			}
		}
	}

	/**
	 * Returns an Iterator over the non-zero entries of the belief state
	 * @return
	 */
	public Collection<Entry<Integer,Double>> getNonZeroEntries(){
		if( ( m_mNonZeroEntries == null ) && ( m_aStateProbabilities != null ) ){
			m_mNonZeroEntries = new StaticMap( m_aStateProbabilities, 0.0 );
			if( m_bSparse )
				m_aStateProbabilities = null;
		}
		return m_mNonZeroEntries;
	}

	@Override
	public Iterator<Entry<Integer, Double>> getDominatingNonZeroEntries() {
		// TODO Auto-generated method stub
		return null;
	}

	public int getNonZeroEntriesCount() {
		getNonZeroEntries();
		return m_mNonZeroEntries.size();
	}
}
