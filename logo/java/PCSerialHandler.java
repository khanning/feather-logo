
public class PCSerialHandler extends SerialHandler {

  static final String[] parityLetter = {"N", "E", "O", "M", "S"};
  static {if(System.getProperty("os.arch").toLowerCase().equals("amd64")) System.loadLibrary("javaserial64-v2");
          else System.loadLibrary("javaserial-v2");}

  boolean openPort (String portname) {
    closePort();
    return nOpenPort(portname)!=0;
  }

  void closePort () {
    nClosePort();
  }

  String getPortName(int pid, int vid){
		String fname = nGetPortName(pid, vid);
		int s = fname.lastIndexOf('(');
		int e = fname.lastIndexOf(')');
		if(s==-1) return "";
		if(e==-1) return "";
		return "\\\\.\\"+fname.substring(s+1,e);
	}


  int readbyte() {return nReadbyte ();}
  void clearcom() {nClearcom ();}
  void writebyte(int b) {nWritebyte (b);}
	void writebytes(byte[] arr){nWritebytes(arr);}
  void usbInit() {nUsbInit ();}
  int portHandle() {return nPortHandle ();}

  void setSerialPortParams(int baud, int databits,
                                  int stopbits, int parity){
    String control = "baud="+baud+" parity="+parityLetter[parity];
    control += " data="+databits+" stop="+stopbits;
    nSetCommState(control);
  }

  static native int nOpenPort(String s);
  static native boolean nClosePort();
  static native boolean nSetCommState(String control);
  static native int nReadbyte();
  static native boolean nClearcom();
  static native boolean nWritebyte(int b);
  static native boolean nWritebytes(byte[] arr);
  static native boolean nUsbInit();
  static native int nPortHandle();
  static native String nGetPortName(int pid, int vid);
}

