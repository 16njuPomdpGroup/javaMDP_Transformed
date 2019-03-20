/*
 * Created on May 5, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package pomdp.utilities;

import java.io.Serializable;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.utilities.datastructures.PriorityQueueElement;

public abstract class BeliefState extends PriorityQueueElement implements Serializable{

	private static final long serialVersionUID = 8715715835544313266L;
	protected double[][] m_aCachedObservationProbabilities;
	protected int m_cStates;
	private Vector<BeliefState> m_vPredecessors;
	protected Map<BeliefState, Pair<Double, Integer>> m_mProbCurrentGivenPred;
	protected Map<Integer,Pair<BeliefState, Double>>[] m_amSuccessors;
	private Vector<BeliefState> m_vAllSuccessors;
	protected int m_cActions;
	protected int m_cObservations;
	protected boolean m_bCacheBeliefStates;
	protected double m_dComputedValue;
	protected double m_dLastMaxValue;
	protected AlphaVector m_avLastMaxAlpha;
	protected double m_dImmediateReward;
	protected double[] m_adActionImmediateReward;
	protected BeliefStateFactory m_bsFactory;
	protected int m_iMaxBeliefState;
	protected double m_dMaxBelief;
	protected double[] m_aStateProbabilities; // 状态概率
	private boolean marked;
	private int level = 1;
	
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public BeliefState( int cStates, int cActions, int cObservations, int id, boolean bCacheBeliefStates, BeliefStateFactory bsFactory ){
		super();
		m_bCacheBeliefStates = bCacheBeliefStates;
		m_cStates = cStates;
		m_cActions = cActions;
		m_cObservations = cObservations;
		m_bsFactory = bsFactory;

		m_iMaxBeliefState = -1;
		m_dMaxBelief = 0.0;
		
		marked = false;


		if( m_bCacheBeliefStates ){
			m_amSuccessors = new TreeMap[m_cActions];
			for( int iAction = 0 ; iAction < m_cActions ; iAction++ ){
				m_amSuccessors[iAction] = new TreeMap( );
			}
		}
		else{
			m_amSuccessors = null;
		}
		m_aCachedObservationProbabilities = new double[m_cActions][m_cObservations];
		for( int i = 0 ; i< m_cActions ; i++ ){
			for( int j = 0 ; j< m_cObservations ; j++ ){
				m_aCachedObservationProbabilities[i][j] = -1.0;
			}
		}

		m_vPredecessors = new Vector<BeliefState>();

		m_dComputedValue = 0;
		m_mProbCurrentGivenPred = new TreeMap<BeliefState, Pair<Double, Integer>>( getComparator() );

		m_dLastMaxValue = 0.0;
		m_avLastMaxAlpha = null;

		m_adActionImmediateReward = new double[m_cActions];
		for( int iAction = 0 ; iAction < m_cActions ; iAction++ ){
			m_adActionImmediateReward[iAction] = Double.NEGATIVE_INFINITY;
		}
		m_dImmediateReward = Double.NEGATIVE_INFINITY;


		m_vAllSuccessors = new Vector<BeliefState>();

	}
	
	public int getLevel()
	{
		return level;
	}
	public void setLevel(int level)
	{
		this.level = level;
	}
	public boolean isMarked()
	{
		return marked;
	}
	
	public void setMarked()
	{
		marked = true;
	}
	
	public void setUnmarked()
	{
		marked = false;
	}
	
	public double[] getStatePro()
	{
		return m_aStateProbabilities;
	}
	
	public void setMaxAlpha( AlphaVector avMax){
		m_avLastMaxAlpha = avMax;
	}
	
	public void setMaxValue( double dMaxValue){
		m_dLastMaxValue = dMaxValue;
	}
	
	public double getActionImmediateReward( int iAction ){
		return m_adActionImmediateReward[iAction];
	}
	
	public void setActionImmediateReward( int iAction, double dReward ){
		m_adActionImmediateReward[iAction] = dReward;
	}
	
	
	
	public BeliefStateFactory getBeliefStateFactory(){
		return m_bsFactory;
	}

	public synchronized void addPredecessor( BeliefState bs, double dProb, int iAction ){
		if( m_bCacheBeliefStates && !m_vPredecessors.contains( bs ) ){
			m_vPredecessors.add( bs );
			m_mProbCurrentGivenPred.put( bs, new Pair<Double, Integer>( new Double( dProb ), new Integer( iAction ) ) );
		}
	}
	
	protected Comparator<BeliefState> getComparator() {
		return BeliefStateComparator.getInstance();
	}

	
	/**
	 * 计算出下一个信念点
	 * @param iAction
	 * @param iObservation
	 * @return
	 */
	public synchronized BeliefState nextBeliefState( int iAction, int iObservation ){
		BeliefState bsNext = null;
		if( m_bCacheBeliefStates && getBeliefStateFactory().isCachingBeliefStates() ){

			Integer iKey = new Integer( iObservation );
			Pair<BeliefState, Double> pEntry = m_amSuccessors[iAction].get( iKey );
			if( pEntry == null ){
				bsNext = getBeliefStateFactory().nextBeliefState( this, iAction, iObservation );
				if( ( bsNext != null ) && ( getBeliefStateFactory().isCachingBeliefStates() ) )
					addSuccessor( iAction, iObservation, bsNext );//添加到后继点集合中
			}
			else{
				bsNext = (BeliefState) pEntry.m_first;
			}
		}
		else{
			bsNext = getBeliefStateFactory().nextBeliefState( this, iAction, iObservation );
		}
		return bsNext;
	}

	public synchronized void addSuccessor( int iAction, int iObservation, BeliefState bsSuccessor ){
		Integer iKey = new Integer( iObservation );
		double dProb = probabilityOGivenA( iAction, iObservation );
		Pair<BeliefState, Double> pEntry = new Pair<BeliefState, Double>( bsSuccessor, new Double( dProb ) );
		m_amSuccessors[iAction].put( iKey, pEntry );
		if( !m_vAllSuccessors.contains( bsSuccessor ) )
			m_vAllSuccessors.add( bsSuccessor );
	}
	
	/*
	 * 当前信念点在执行动作a，观察到o的概率
	 * 等于b到b(a,z)的转移概率
	 * 调用规则：bs.probabilityOGivenA(a, o)
	 */
	public double probabilityOGivenA( int iAction, int iObservation ){
		if( m_aCachedObservationProbabilities == null )
			return getBeliefStateFactory().calcNormalizingFactor( this, iAction, iObservation );
		double dValue = m_aCachedObservationProbabilities[iAction][iObservation];
		if( dValue < 0.0 ){
			dValue = getBeliefStateFactory().calcNormalizingFactor( this, iAction, iObservation );
			m_aCachedObservationProbabilities[iAction][iObservation] = dValue;
		}
		return dValue;
	}
	
	public void setProbabilityOGivenA( int iAction, int iObservation, double dValue ){
		if( m_aCachedObservationProbabilities != null )
			m_aCachedObservationProbabilities[iAction][iObservation] = dValue;
	}
	
	public abstract double valueAt( int iState );

	public abstract void setValueAt( int iState, double dValue );

	public int getMostLikelyState(){
		return m_iMaxBeliefState;
	}
	public abstract long size();
	/**
	 * Returns an Iterator over the non-zero entries of the belief state
	 * @return
	 */
	public abstract Collection<Entry<Integer,Double>> getNonZeroEntries();

	public abstract Iterator<Entry<Integer, Double>> getDominatingNonZeroEntries();
	public void release() {
	}
	public abstract int getNonZeroEntriesCount();
	
}
