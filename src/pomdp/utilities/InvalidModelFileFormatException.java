
package pomdp.utilities;

public class InvalidModelFileFormatException extends Exception {
	
	private static final long serialVersionUID = 1L;
	
	public InvalidModelFileFormatException(){
		super();
	}
	public InvalidModelFileFormatException( String sData ){
		super( sData );
	}
}
