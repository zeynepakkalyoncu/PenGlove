import processing.serial.*;
import java.io.File;
import net.sourceforge.tess4j.*;

Serial port;
int tab = 9;
float posx, posy, curx, cury, inx, iny, finx, finy;
int started = 0;

void setup(){
  String portName = Serial.list()[0];
  port = new Serial(this, portName, 115200);  
  size(1000, 500);
  background(65);
  posx = 0;
  posy = 0;
  curx = 250;
  cury = 0;
}

void draw(){
  int m = millis();
  stroke(255);
  if (port.available() > 0){
    String tempx = port.readStringUntil(tab);
    String tempy = port.readStringUntil(tab);
    if ((tempx != null) && (tempy != null) && (m > 2000)){
      posx = float(tempx); posy = float(tempy);
      println(posx + " , " + posy);
      strokeWeight(5); 
      line(curx + 250, cury + 250, posx + 250, posy + 250);
      curx = posx; cury = posy;
      if (mousePressed == true){
        saveFrame("letter.png");
        File imageFile = new File("letter.png");
        Tesseract instance = Tesseract.getInstance();
        try{
          String result = instance.doOCR(imageFile);
          textSize(28);
          text(result, 30, 40);
        }catch (TesseractException e){
          println(e.getMessage());
        }
      }
    }
  }
}
