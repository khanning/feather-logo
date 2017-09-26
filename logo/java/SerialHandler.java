
public abstract class SerialHandler {

  boolean openPort (String s) {return false;}
  boolean reopenPort() {return false;}
  String getPortName(int pid, int vid) {return null;}
  void closePort () {}
  int readbyte () {return 0;}
  void clearcom () {}
  void writebyte (int b) {}
  void writebytes (byte[] arr) {}
	int portHandle() {return 0;}
  void usbInit () {}

  void setSerialPortParams(int baud, int databits,
                                  int stopbits, int parity){};
}

