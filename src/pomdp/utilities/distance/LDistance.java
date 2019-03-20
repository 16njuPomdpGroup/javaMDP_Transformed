package pomdp.utilities.distance;

import java.util.Iterator;
import java.util.Map.Entry;
import java.util.TreeMap;

import pomdp.utilities.BeliefState;

public abstract class LDistance implements DistanceMetric
{
	protected TreeMap<?, ?> m_tmCache;
	
	public LDistance()
	{
		m_tmCache = null;
	}
	
	protected double getInitialDistance()
	{
		return 0.0;
	}
	
	//求两个信念点之间的距离
	public double distance(BeliefState bs1, BeliefState bs2)
    {
    	double dDistance = -1.0;
		
		if( bs1 == bs2 )
		{
			dDistance = 0.0;
		}
		else
		{
			Iterator<Entry<Integer, Double>> itFirstNonZero = bs1.getNonZeroEntries().iterator();
			Iterator<Entry<Integer, Double>> itSecondNonZero = bs2.getNonZeroEntries().iterator();
			
			int iState1 = -1, iState2 = -1;
			
			double dValue1 = 0.0, dValue2 = 0.0;
			Belief b = null;
			
			dDistance = getInitialDistance();//先获得初始距离
			
			//|(|b_1-b_2 |)|_1=∑_(s∈S)|b_1 (s)-b_2 (s)|
			while((iState1<Integer.MAX_VALUE) || (iState2<Integer.MAX_VALUE))
			{
				if( iState1 == iState2 )
				{					
					dDistance = applyDistanceMetric( dDistance, dValue1, dValue2 );//dDistance+Math.abs(dValue1, dValue2)
					
					b = new Belief(itFirstNonZero);
					iState1 = b.iState;
					dValue1 = b.dValue;
					b = new Belief( itSecondNonZero );
					iState2 = b.iState;
					dValue2 = b.dValue;
				}
				else if( iState1 < iState2 )
				{
					dDistance = applyDistanceMetric( dDistance, dValue1, 0 );

					b = new Belief( itFirstNonZero );
					iState1 = b.iState;
					dValue1 = b.dValue;				
				}
				else if( iState2 < iState1 )
				{
					dDistance = applyDistanceMetric( dDistance, dValue2, 0 );

					b = new Belief( itSecondNonZero );
					iState2 = b.iState;
					dValue2 = b.dValue;	
				}
			}
		}
		dDistance = applyFinal(dDistance);
    	
    	return dDistance;
    }
	
	protected abstract double applyDistanceMetric( double dAccumulated, double dValue1, double dValue2 );
	
	protected abstract double applyFinal( double dAccumulated );
	
	
	
	/**
	 * 状态--概率
	 */
	public class Belief
	{
		public int iState;
		public double dValue;
		
		public Belief( Iterator<Entry<Integer, Double>> it )
		{
			if( it.hasNext() )
			{
				Entry<Integer, Double> e = (Entry<Integer, Double>)it.next();
				iState = ((Integer) e.getKey()).intValue();
				dValue = ((Double) e.getValue()).doubleValue();
			}
			else
			{
				iState = Integer.MAX_VALUE;
				dValue = -1.0;
			}
		}
	}
}







