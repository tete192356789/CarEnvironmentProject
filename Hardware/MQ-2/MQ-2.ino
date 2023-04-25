


int analogPin = 35; //ประกาศตัวแปร ให้ analogPin แทนขา analog ขาที่5
int val = 0;
void setup() {

Serial.begin(9600);
}
void loop() {
val = analogRead(analogPin); //อ่านค่าสัญญาณ analog ขา5
Serial.print("val = "); // พิมพ์ข้อมความส่งเข้าคอมพิวเตอร์ "val = "
Serial.println(val); // พิมพ์ค่าของตัวแปร val

delay(100);
}