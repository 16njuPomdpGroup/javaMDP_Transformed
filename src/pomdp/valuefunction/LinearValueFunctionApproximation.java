package pomdp.valuefunction;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.Serializable;
import java.util.Iterator;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.datastructures.LinkedList;

public class LinearValueFunctionApproximation extends PolicyStrategy implements Serializable {

	private static final long serialVersionUID = 1L;
	
	protected LinkedList<AlphaVector> m_vAlphaVectors;
	protected RandomGenerator m_rndGenerator;
	
	protected double m_dEpsilon;
	protected boolean m_bCacheValues;
	protected double m_dMaxValue;
	
	public LinearValueFunctionApproximation( double dEpsilon, boolean bCacheValues ){
		m_vAlphaVectors = new LinkedList<AlphaVector>();
		m_dEpsilon = dEpsilon;
		m_bCacheValues = true;
		m_dMaxValue = 0.0;
		m_rndGenerator = new RandomGenerator( "LinearValueFunctionApproximation" );
	}

	public LinearValueFunctionApproximation(){
		this( 1.0, true );
	}

	@Override
	public int getAction(BeliefState bsCurrent) {
		return getBestAction(bsCurrent);
	}
	
	public AlphaVector getLast() {
		return m_vAlphaVectors.getLast();
	}
	
	public int getBestAction( BeliefState bs ){
		AlphaVector avMaxAlpha = getMaxAlpha( bs );
		if( avMaxAlpha == null )
			return -1;
		return avMaxAlpha.getAction();
	}
	
	private void addVector( AlphaVector avNew ){
		m_vAlphaVectors.add( avNew );
	}
	
	public void add( AlphaVector avNew ){
		add( avNew, false );
	}
	
	public void add( AlphaVector avNew, boolean bPruneDominated ){
		AlphaVector avExisting = null;
		BeliefState bsWitness = null;
		boolean bDominated = false;
		double dNewValue = 0.0;
		
		if( bPruneDominated ){
			int iVector = 0;
			for( iVector = 0 ; iVector < m_vAlphaVectors.size() && !bDominated ; iVector++ ){
				avExisting = m_vAlphaVectors.get( iVector );
				if( avNew.dominates( avExisting ) ){
					m_vAlphaVectors.remove( avExisting );
				}
				else if( avExisting.dominates( avNew ) ){
					bDominated = true;
				}
			}
		}
		
		if( !bDominated ){
			m_vAlphaVectors.add( avNew );
		
			if( m_bCacheValues ){		
				bsWitness = avNew.getWitness();
				if( bsWitness != null ){
					dNewValue = avNew.dotProduct( bsWitness );
					bsWitness.setMaxAlpha( avNew);
					bsWitness.setMaxValue( dNewValue);
				}
			}
	
			if( avNew.getMaxValue() > m_dMaxValue )
				m_dMaxValue = avNew.getMaxValue();
		}
		
	}
	public AlphaVector elementAt(int iElement ){
		return (AlphaVector) m_vAlphaVectors.get( iElement );
	}
	
	public int size(){
		return m_vAlphaVectors.size();
	}
	
	public void load( String sFileName, POMDP pomdp ) throws Exception{
		DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
		Document docValueFunction = builder.parse( new FileInputStream( sFileName ) );
		Element eValueFunction = (Element)docValueFunction.getChildNodes().item( 0 );
		parseDOM( eValueFunction, pomdp );
	}
	
	public void save( String sFileName ) throws Exception{
		Document docValueFunction = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
		Element eValueFunction = getDOM( docValueFunction );
		
		// Use a Transformer for output
		TransformerFactory tFactory = TransformerFactory.newInstance();
		Transformer transformer = tFactory.newTransformer();
		
		DOMSource source = new DOMSource( eValueFunction );
		StreamResult result = new StreamResult( new FileOutputStream( sFileName ) );
		transformer.transform( source, result );
	}
	
	public AlphaVector getMaxAlpha( BeliefState bs ){
		AlphaVector avMaxAlpha = null;
		double maxValue = Double.NEGATIVE_INFINITY;//先把maxValue初始化为负无穷
		
		Iterator<AlphaVector> iter =  m_vAlphaVectors.iterator();//m_vAlphaVectors.backwardIterator()没有此方法？
		while(iter.hasNext())//求每一个向量与bs的乘积
		{
			AlphaVector avCurrent = iter.next();
			double value = avCurrent.dotProduct(bs);
			
			if(value >= maxValue)//保存最大值和最大向量
			{
				maxValue = value;
				avMaxAlpha = avCurrent;
			}
		}
		
		if(avMaxAlpha != null)//设置信念点的最大值和最大向量
		{
			bs.setMaxValue(maxValue);
			bs.setMaxAlpha(avMaxAlpha);
		}
		return avMaxAlpha;
	}
	
	public boolean addPrunePointwiseDominated( AlphaVector avNew ){
		Iterator<AlphaVector> iter = m_vAlphaVectors.iterator();
		while(iter.hasNext())//遍历值函数中的向量
		{
			AlphaVector avExisting = iter.next();
			if( avExisting.equals( avNew ) || avExisting.dominates( avNew ) ){//avNew存在或者avNew被统治则返回false
				return false;
			}
			else if( avNew.dominates( avExisting ) ){//如果avNew统治值函数中的某一个向量，把这个向量删除
				iter.remove();
			}
		}
		
		addVector( avNew );//把avNew加入到值函数中
		
		if( m_bCacheValues ){		
			BeliefState bsWitness = avNew.getWitness();
			if( bsWitness != null ){
				double dNewValue = avNew.dotProduct( bsWitness );
				bsWitness.setMaxAlpha( avNew);
				bsWitness.setMaxValue( dNewValue);
			}
		}
		
		if( avNew.getMaxValue() > m_dMaxValue )
			m_dMaxValue = avNew.getMaxValue();
		return true;
	}
	
	public void clear() {
		for( AlphaVector av : m_vAlphaVectors ){
			av.release();
		}
		m_vAlphaVectors.clear();
	}
	
	public Element getDOM( Document doc ) throws Exception{
		Element eValueFunction = doc.createElement( "ValueFunction" ), eAlphaVector = null;
		AlphaVector avCurrent = null;
		
		eValueFunction = doc.createElement( "ValueFunction" );
		eValueFunction.setAttribute( "AlphaVectorCount", m_vAlphaVectors.size() + "" );
		eValueFunction.setAttribute( "Epsilon", m_dEpsilon + "" );
		eValueFunction.setAttribute( "CacheValue", m_bCacheValues + "" );
		eValueFunction.setAttribute( "MaxValue", m_dMaxValue + "" );		
		doc.appendChild( eValueFunction );
		
		int iVector = 0;
		for( iVector = 0 ; iVector < m_vAlphaVectors.size() ; iVector++ ){
			avCurrent = m_vAlphaVectors.get( iVector );
			eAlphaVector = avCurrent.getDOM( doc );
			eValueFunction.appendChild( eAlphaVector );
		}
		
		return eValueFunction;
	}
	
	public void parseDOM( Element eValueFunction, POMDP pomdp ) throws Exception{
		Element eVector = null;
		NodeList nlVectors = null;
		int cVectors = 0, iVector = 0;
		AlphaVector avNew = null;
		
		cVectors = Integer.parseInt( eValueFunction.getAttribute( "AlphaVectorCount" ) );
		nlVectors = eValueFunction.getChildNodes();
		
		m_dEpsilon = Double.parseDouble( eValueFunction.getAttribute( "Epsilon" ) );
		m_bCacheValues = Boolean.parseBoolean( eValueFunction.getAttribute( "CacheValue" ) );
		m_dMaxValue = Double.parseDouble( eValueFunction.getAttribute( "MaxValue" ) );

		for( iVector = 0 ; iVector < cVectors ; iVector++ ){
			eVector = (Element)nlVectors.item( iVector );
			avNew = AlphaVector.parseDOM( eVector, pomdp );
			m_vAlphaVectors.add( avNew );
		}
	}
}
