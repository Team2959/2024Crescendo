package cwtech.util;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;

public class Bling
{
	public enum BlingMessage
	{
		Off,
		Start,
		Red,
		Green,
		Blue
	};

	private final SerialPort m_serialPort = new SerialPort(9600, SerialPort.Port.kOnboard);
	private BlingMessage m_lastMessage = BlingMessage.Off;
	
	public Bling()
	{
		m_serialPort.reset();
		m_serialPort.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
		setBlingMessage(BlingMessage.Start);
	}
	
	public void setBlingMessage(BlingMessage message)
	{
		if (message == m_lastMessage)
			return;
		m_lastMessage = message;
		
		String command;
		switch (message)
		{
			case Start:
				command = "S!";
				break;
			case Red:
				command = "R!";
				break;
			case Green:
				command = "G!";
				break;
			case Blue:
				command = "B!";
				break;
			case Off:
			default:
				command = "O!";
				break;
		}
		
		m_serialPort.writeString(command);
	}
	
	public void setFlashState(boolean flash)
	{
		m_serialPort.writeString(flash ? "F!" : "N!");
	}
}