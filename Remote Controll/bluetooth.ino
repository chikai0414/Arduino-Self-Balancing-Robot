#include <SoftwareSerial.h> // 引用程式庫
// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳
char val; // 儲存接收資料的變數
String s;
void setup() {
Serial.begin(57600); // 與電腦序列埠連線
//Serial.println("BT is ready!");
// 設定藍牙模組的連線速率
// 如果是HC05，請改成38400
// 如果是HC06，請改成9600
BT.begin(57600);
}
void loop() {

  if ((BT.available())) {
    Serial.write(BT.read());  //把藍芽的東西印上Com
    //s=BT.readString();
    //Serial.println(s);
    //Serial.print("\n");
    //delay(20);
  }
  if (Serial.available()) {   
   // BT.write(Serial.read());
    val=Serial.read();    //讀取Serial
    BT.write(val);        //用藍芽傳出去
    //Serial.write(val);
  }
}
