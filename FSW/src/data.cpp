
#include "data.h"

void msg_TIME(FSW_SYSTEM fsw){
    String rs;
}

void msg_EPDS(FSW_SYSTEM fsw){
    String rs;
    rs = rs;
    rs += String(fsw.ads.quaternion.q1) +";"+ String(fsw.ads.quaternion.q2) +";"+ String(fsw.ads.quaternion.q3) +";"+String(fsw.ads.quaternion.q4)+";";
    rs += String(fsw.ads.bfield.x) + ";" + String(fsw.ads.bfield.y) + ";" + String(fsw.ads.bfield.z) + ";";
    rs +=
}

void msg_FSW(FSW_SYSTEM fsw){
    
}

void msg_ADS(){


}


String msg_AI(FSW_SYSTEM fsw){
String rs = "";
rs += String(fsw.AI.prediction) + ";" + String(fsw.AI.probability) + '\n';
return rs;
}


String msg_SPEC(FSW_SYSTEM fsw){
    String rs = "";
    for (int i = 0; i<4096; i++;){
        rs += String(fsw.OpenGammaData.bins[i]) + ";";
    }
    rs += + '\n';
    return rs;
}