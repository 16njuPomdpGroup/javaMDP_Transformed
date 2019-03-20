package pomdp.utilities.distance;

public class L1Distance extends LDistance
{
    protected static L1Distance m_l1Distance;
	
	public L1Distance()
	{
		super();
	}
	
	public static DistanceMetric getInstance()
	{
		if( m_l1Distance == null )
		{
			m_l1Distance = new L1Distance();
		}
		return m_l1Distance;
	}
    
	protected double applyDistanceMetric(double dAccumulated, double dValue1, double dValue2) 
	{
		return dAccumulated + Math.abs( dValue1 - dValue2 );
	}

	protected double applyFinal(double dAccumulated) 
	{
		return dAccumulated;
	}
    
}
