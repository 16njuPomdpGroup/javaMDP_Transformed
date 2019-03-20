package pomdp.utilities;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import pomdp.utilities.datastructures.Function;

/*
 * 2018-4-29
 * 继承自Function类，用于保存转移函数，值函数，观察函数，初始状态
 */
public class SparseTabularFunction extends Function {

	private static final long serialVersionUID = 1L;
	
	protected HashMap<Integer, Double> m_mSingleParameterValues;
	protected HashMap<Integer, Double>[] m_mDualParameterValues;
	protected HashMap<Integer,Double>[][] m_mTripleParametermValues;
	
	@SuppressWarnings("unchecked")
	public SparseTabularFunction( int[] aiDims ){
		super( aiDims );
		int i = 0, j = 0;
		if( aiDims.length >= 1 ){
			m_mSingleParameterValues = new HashMap<Integer, Double>();
			if( aiDims.length >= 2 ){
				m_mDualParameterValues = new HashMap[aiDims[0]];
				for( i = 0 ; i < aiDims[0] ; i++ )
					m_mDualParameterValues[i] = new HashMap<Integer, Double>();
				if( aiDims.length >= 3 ){
					m_mTripleParametermValues = new HashMap[aiDims[0]][aiDims[1]];
					for( i = 0 ; i < aiDims[0] ; i++ )
						for( j = 0 ; j < aiDims[1] ; j++ )
							m_mTripleParametermValues[i][j] = new HashMap<Integer,Double>();
				}
			}
		}
	}

	public double valueAt( int arg1 ){
		Double dValue = (Double) m_mSingleParameterValues.get( new Integer( arg1 ) );
		if( dValue == null )
			return 0.0;
		return dValue.doubleValue();
	}

	public double valueAt( int arg1, int arg2 ){
		Double dValue = (Double) m_mDualParameterValues[arg1].get( new Integer( arg2 ) );
		if( dValue == null )
			return 0.0;
		return dValue.doubleValue();
	}

	public double valueAt( int arg1, int arg2, int arg3 ){
		Double dValue = m_mTripleParametermValues[arg1][arg2].get( arg3 );
		if( dValue == null )
			return 0.0;
		return dValue.doubleValue();
	}

	public void setValue( int arg1, double dValue ){
		if( dValue > m_dMaxValue )
			m_dMaxValue = dValue;
		if( dValue < m_dMinValue )
			m_dMinValue = dValue;
		if( dValue != 0.0 ){
			m_mSingleParameterValues.put( new Integer( arg1 ), new Double( dValue ) );
		}
		else{
			m_mSingleParameterValues.remove( new Integer( arg1 ) );
		}
	}

	public void setValue( int arg1, int arg2, double dValue ){
		if( dValue > m_dMaxValue )
			m_dMaxValue = dValue;
		if( dValue < m_dMinValue )
			m_dMinValue = dValue;
		if( dValue != 0.0 ){
			m_mDualParameterValues[arg1].put( new Integer( arg2 ), new Double( dValue ) );
		}
		else{
			m_mDualParameterValues[arg1].remove( new Integer( arg2 ) );
		}
	}
	
	/**
	 * ����Transition Function��һ��ת�������� ������iStartState, iActionIdx, iEndState, dValue)
	 */
	public void setValue( int arg1, int arg2, int arg3, double dValue ){
		
		
		if( dValue > m_dMaxValue )
			m_dMaxValue = dValue;
		if( dValue < m_dMinValue )
			m_dMinValue = dValue;
		if( dValue != 0.0 ){
			m_mTripleParametermValues[arg1][arg2].put( arg3, dValue );
		}
		else{
			m_mTripleParametermValues[arg1][arg2].remove( arg3 );
		}
		
	}

	public Iterator<Entry<Integer,Double>> getNonZeroEntries( int arg1, int arg2 ){
		return m_mTripleParametermValues[arg1][arg2].entrySet().iterator();
	}
	
	public Iterator<Entry<Integer, Double>> getNonZeroEntries() {
		return m_mSingleParameterValues.entrySet().iterator();
	}

	public int countNonZeroEntries( int arg1, int arg2 ){
		return m_mTripleParametermValues[arg1][arg2].size();
	}

	// 三维的才可调用此函数
	public int countEntries() {
		int cEntries = 0;
		int i = 0, j = 0;
		for( i = 0 ; i < m_mTripleParametermValues.length ; i++ )
			for( j = 0 ; j < m_mTripleParametermValues[0].length ; j++ )
				cEntries += m_mTripleParametermValues[i][j].size();
		return cEntries;
	}

	public int countNonZeroEntries() {
		return m_mSingleParameterValues.size();
	}
}
