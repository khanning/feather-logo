import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.BorderLayout;
import java.awt.event.KeyEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowAdapter;
import java.awt.Color;
import java.awt.FileDialog;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Insets;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.Hashtable;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.Action;
import javax.swing.AbstractAction;
import javax.swing.text.DefaultEditorKit;
import javax.swing.text.EditorKit;
import javax.swing.text.Keymap;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.text.JTextComponent;


public class LogoCC {
  static LogoConsole logoconsole;
  static LContext lc;
  static Timer timer;
  static String[] primclasses =
    {"SystemPrims", "MathPrims", "ControlPrims",
     "DefiningPrims", "WordListPrims", "FilePrims",
     "SerialPrims", "CCPrims"};

  public static void main(String[] args){
    lc = new LContext();
    Logo.setupPrims(primclasses, lc);
    (logoconsole = new LogoConsole("Logo")).init();
    (new LogoCommandRunner("load \"compiler startup", lc, true)).run();
    lc.tyo.println("Welcome to Logo!");
    logoconsole.cc.requestFocus();
    timer = new Timer();
    timer.scheduleAtFixedRate(new TickTask(lc), 2000, 50);         // Wait 2 seconds before activating the timer
    }
}


class LogoConsole extends JFrame {

  LogoButtonListener bl;
  JPanel buttons;
  Listener cc;
  JTextArea monitor;
  static JTextComponent status, filename;

  LogoConsole(String s){super(s);}

  void init() {
    // Create 2 panels
    JPanel lcpanel = new JPanel();
    lcpanel.setLayout(new BorderLayout());
    getContentPane().add(lcpanel, BorderLayout.CENTER);

    JPanel monitorpanel = new JPanel();
//    monitorpanel.setLayout(new BorderLayout());
//    getContentPane().add(monitorpanel, BorderLayout.EAST);

    cc=new Listener(20, 60);
    JScrollPane scrollpane = new JScrollPane(cc,
              JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
              JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
    lcpanel.add(scrollpane, BorderLayout.CENTER);
 //  scrollpane.validate();
    cc.prefix = "command-center-cr ["; cc.suffix="]";
    cc.setFont(new Font("Courier", Font.PLAIN, 16));
    cc.setMargin (new Insets(0, 5, 0, 0));
    cc.setWrapStyleWord(true);
    cc.setLineWrap(true);
    cc.setForeground(new Color(.2f,0f,.5f));
    initAppleKeys(cc);

    buttons = new JPanel();
    lcpanel.add(buttons, BorderLayout.SOUTH);
    buttons.setLayout(new FlowLayout(FlowLayout.LEFT));
    bl = new LogoButtonListener(cc);
    addButton("download", "download");

    filename = new JTextField("test.txt", 12);
    buttons.add(filename);

    JButton browse = new JButton("...");
    buttons.add(browse);
    browse.addActionListener(new ActionListener (){
      public void actionPerformed(ActionEvent e) {
        String[] result = getFileName(LogoCC.logoconsole, "logo file");
        filename.setText(result[1]);
        cc.lc.dirname = result[0];
        }
    });

    status = new JTextField(12);
    buttons.add(status);

    addButton("stop", "stopvm");

    pack(); setVisible(true);
    addWindowListener(new ToplevelWindowListener() );
    cc.addKeyListener(new TEKeyListener() );

    cc.lc = LogoCC.lc;
    cc.lc.tyo = new PrintWriter(new TEStream(cc), true);
    cc.lc.filebox = filename;
    cc.lc.status = status;

    cc.lc.montyo = new PrintWriter(new TEStream(monitor), true);

  }

    public void initAppleKeys(JTextArea text) {
    // define the actions
    DefaultEditorKit.CutAction cutaction = new DefaultEditorKit.CutAction ();
    DefaultEditorKit.CopyAction copyaction = new DefaultEditorKit.CopyAction ();
    DefaultEditorKit.PasteAction pasteaction = new DefaultEditorKit.PasteAction ();
    SelectAllAction selectallaction = new SelectAllAction();


    // add the keymap to object
    Keymap keymap = text.getKeymap();
  // map the key strokes
  KeyStroke apple_x = KeyStroke.getKeyStroke(KeyEvent.VK_X, java.awt.event.InputEvent.META_MASK);
  KeyStroke apple_c = KeyStroke.getKeyStroke(KeyEvent.VK_C, java.awt.event.InputEvent.META_MASK);
  KeyStroke apple_v = KeyStroke.getKeyStroke(KeyEvent.VK_V, java.awt.event.InputEvent.META_MASK);
  KeyStroke apple_a = KeyStroke.getKeyStroke(KeyEvent.VK_A, java.awt.event.InputEvent.META_MASK);
  keymap.addActionForKeyStroke(apple_x, cutaction);
  keymap.addActionForKeyStroke(apple_c, copyaction);
  keymap.addActionForKeyStroke(apple_v, pasteaction);
  keymap.addActionForKeyStroke(apple_a, selectallaction);
  text.setKeymap(keymap);
  }

  void addButton(String s, String name){
    JButton button;
    button = new JButton(s);
    button.setName(name);
    button.addActionListener(bl);
    buttons.add(button);
  }

  static String currentdir = "";

  static String[] getFileName(JFrame frm, String s){
   FileDialog f = new FileDialog(frm, s);
   f.setDirectory(currentdir);
   f.setVisible(true);
   currentdir = f.getDirectory();
   String[] result = new String[2];
   result[0] = f.getDirectory();
   result[1] = f.getFile();
   f.dispose();
   return result;
 }

}

class TickTask extends TimerTask {

  LogoCommandRunner runticktask;

  TickTask(LContext lc){
    super();
    runticktask = new LogoCommandRunner("ticktask", lc, true);
    }

  public void run() {
       runticktask.run();
    }

}

class LogoButtonListener implements ActionListener
{
  Listener cc;
  LogoButtonListener(Listener cc){this.cc = cc;}

  public void actionPerformed(ActionEvent e)
  {
    String label = ((JButton) e.getSource()).getName();
    cc.runSilent(label);
  }
}

class ToplevelWindowListener extends WindowAdapter
{
  public void windowClosing(WindowEvent e)
  {
    System.exit(0);
    }
}

class TEKeyListener extends KeyAdapter
{
  public void keyPressed(KeyEvent e){
    Listener cc = (Listener) e.getComponent();
    char key = e.getKeyChar();
    int code = e.getKeyCode();                          // Patch so Ctrl + C is not interpreted as a return
    if(key== '\n' && code == 10){cc.handlecr(); e.consume(); return;}
    if(key=='\u0001'){cc.selectAll(); e.consume(); return;}
    if (key=='\u0012') {cc.runLine("reload"); e.consume(); return;} // ctrl+r
    if (key=='\u001b') {cc.lc.timeToStop=true; e.consume(); return;} // escape
    }
}

class Listener extends JTextArea {

  LContext lc;
  String file, dir;
  String prefix="", suffix="";

  Listener(int h, int w){super(h,w);};

  void handlecr(){
    String s=getText();
    int sol=findStartOfLine(s, getCaretPosition());
    int eol=findEndOfLine(s, sol);
    if(eol==s.length()) append("\n");
    setCaretPosition(eol+1);
    if(!prefix.equals("")) runSilent(prefix+s.substring(sol, eol)+suffix);
    else runLine(s.substring(sol, eol));
  }

  int findStartOfLine(String s, int i){
    int val = s.lastIndexOf(10,i-1);
    if (val<0) return 0;
    return val+1;
  }

  int findEndOfLine(String s, int i){
    int val = s.indexOf('\n',i);
    if (val<0) return s.length();
    return val;
  }

  void runLine(String s){
    (new Thread(new LogoCommandRunner(s, lc))).start();
  }

  void runSilent(String s){
   (new Thread(new LogoCommandRunner(s, lc, true))).start();
  }

}

class TEStream extends OutputStream {

  JTextArea te;
  String buffer = "";

  public TEStream(JTextArea te){this.te = te;}

  public void write(int n) {
    if (n==10) buffer+='\n';
    else if (n==13) return;
    else buffer+=(char)n;
  }

  public void flush(){
    int pos = te.getCaretPosition();
    te.insert(buffer, pos);
    te.setCaretPosition(pos+buffer.length());
    buffer="";
  }
}

class CCPrims extends Primitives {

  static String[] primlist={
    "print", "1", "setstatus", "1", "status", "0",
    "setfile-field", "1", "file-field", "0", "dirname", "0",
    "monprint", "1", "moninsert", "1"
};

  public String[] primlist(){return primlist;}

  public Object dispatch(int offset, Object[] args, LContext lc){
    switch(offset){
    case 0: return prim_print(args[0], lc);
    case 1: return prim_setstatus(args[0], lc);
    case 2: return prim_status(lc);
    case 3: return prim_setfilename(args[0], lc);
    case 4: return prim_filename(lc);
    case 5: return prim_dirname(lc);
    case 6: return prim_monprint(args[0], lc);
    case 7: return prim_moninsert(args[0], lc);
 }
    return null;
  }

  Object prim_print(Object arg1, LContext lc){
    lc.tyo.println(Logo.prs(arg1));
    return null;
  }

  Object prim_setstatus(Object arg1, LContext lc){
    lc.status.setText(Logo.prs(arg1));
    return null;
  }

  Object prim_status(LContext lc){
    return lc.status.getText();
  }

  Object prim_setfilename(Object arg1, LContext lc){
    lc.filebox.setText(Logo.prs(arg1));
    return null;
  }

  Object prim_filename(LContext lc){
    return lc.dirname + lc.filebox.getText();
  }

  Object prim_dirname(LContext lc){
    return lc.dirname;
  }

  Object prim_monprint(Object arg1, LContext lc){
    lc.montyo.println(Logo.prs(arg1));
    return null;
  }

  Object prim_moninsert(Object arg1, LContext lc){
    lc.montyo.print(Logo.prs(arg1));
    return null;
  }

}


//--------------- Mac Key handling classes -----------------

class SelectAllAction extends AbstractAction implements Action {

    public void actionPerformed(ActionEvent e) {
      JTextArea text = (JTextArea)e.getSource();
      text.selectAll();
     // e.consume();
    }
}

//-----------------------------------------------------------
// file

 class LContext {
   Hashtable<String,Symbol> oblist = new Hashtable<String,Symbol>();
   Hashtable<Object,Hashtable<Object,Object>> props = new Hashtable<Object,Hashtable<Object,Object>>();
   MapList iline;
   Symbol cfun, ufun;
   Object ufunresult, juststop = new Object();
   boolean mustOutput, timeToStop;
   int priority = 0;
   Object[] locals;
   String errormessage;
   PrintWriter tyo, montyo;
   Thread thread;
   String filename;
   boolean loadFromResource = false;
   String dirname="";
   JTextComponent status, filebox;
   Listener cc;
 }


