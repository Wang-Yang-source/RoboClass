// 定义常量与变量
const int triggerPin = 2;
const int echoPin = A1; //超声波引脚定义
double distance;

// 定义Ultrasonic类
class Ultrasonic
{
public:
  Ultrasonic(int trigger, int echo)
  {
    this->trigger = trigger;
    this->echo = echo;

    pinMode(this->trigger, OUTPUT);
    pinMode(this->echo, INPUT);
  }
  ~Ultrasonic() {}

  double getDistance()
  {
    digitalWrite(this->trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigger, LOW);

    long duration = pulseIn(this->echo, HIGH);
    return duration / 58.2;
  }

private:
  int trigger;
  int echo;
};

Ultrasonic ultrasonic(triggerPin, echoPin);

void test_ultra()
{
    distance = ultrasonic.getDistance(); // 获取超声波测量的距离
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);
}

double getDistance()
{
    return distance;
}
