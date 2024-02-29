package cwtech.util;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Bling
{
	public enum BlingMessage
	{
		Off,
		Start,
		Red,
		Green
	};

	private final SerialPort m_serialPort = new SerialPort(9600, SerialPort.Port.kOnboard);
	private BlingMessage m_lastMessage = BlingMessage.Off;
	
	public Bling()
	{
		m_serialPort.reset();
		m_serialPort.WriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
		setBlingMessage(BlingMessage.Start);
	}
	
	public void setBlingMessage(BlingMessage message)
	{
		if (message == m_lastMessage)
			return;
		
		string command;
		switch (message)
		{
			case Off:
				message = "O";
				break;
			case Start:
				message = "S";
				break;
			case Red:
				message = "R";
				break;
			case Green:
				message = "G";
				break;
		}
		
		m_serialPort.writeString(command);
	}
}