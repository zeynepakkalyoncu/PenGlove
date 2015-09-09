import processing.serial.*;

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
  stroke(255);
  if (port.available() > 0){
    String tempx = port.readStringUntil(tab);
    String tempy = port.readStringUntil(tab);
    if ((tempx != null) && (tempy != null)){
      
  /*    posx = -float(tempx); posy = float(tempy);
      // NaN input
      if(!Float.isNaN(posx) && !Float.isNaN(posy)){
        println(posx + " , " + posy);
        strokeWeight(4); 
        line(curx+250, cury+250, posx+250, posy+250);
        curx = posx; cury = posy;
      }
      */
      posx = float(tempx); posy = float(tempy);
      println(posx + " , " + posy);
      strokeWeight(5); 
    /*  if (m >= 3000){
        // new letter
        started = 0;
        finx = 
      }
      else{
        line(curx + 250, cury + 250, posx + 250, posy + 250);
        started = 1;
        inx = curx +250; iny = cury+250;
        curx = posx; cury = posy;
        
      }*/
      line(curx + 250, cury + 250, posx + 250, posy + 250);
        curx = posx; cury = posy;
    }
  }
}
