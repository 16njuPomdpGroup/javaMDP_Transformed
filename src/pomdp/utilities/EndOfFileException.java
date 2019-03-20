package pomdp.utilities;
import java.io.IOException;
 
public class EndOfFileException extends IOException {
	
	private static final long serialVersionUID = 1L;
	

	public EndOfFileException(){
		super();
	}

	public EndOfFileException( String sData ){
		super( sData );
	}
}
