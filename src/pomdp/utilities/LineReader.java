//用来读取文件
package pomdp.utilities;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class LineReader{
	BufferedReader m_fosInput;
	public LineReader( String sFileName ) throws IOException{
		m_fosInput = new BufferedReader( new FileReader( sFileName ) );
	}
	
	public String readLine() throws IOException, EndOfFileException{
		String sLine = m_fosInput.readLine();
		return sLine;
	}
	
	public boolean endOfFile() throws IOException{
		return !m_fosInput.ready();
	}
}
