// Định nghĩa các chân kết nối
const int ENA = 9;      // Chân PWM điều khiển tốc độ động cơ
const int IN1 = 7;      // Chân điều khiển chiều quay động cơ
const int IN2 = 8;
const int encoderPinA = 2;    // Chân encoder A
const int encoderPinB = 3;    // Chân encoder B

// Biến PID
double Kp = 0, Ki = 0, Kd = 0;
double setpoint = 120;        // Tốc độ mục tiêu (RPM)
double currentSpeed = 0;      // Tốc độ hiện tại của động cơ
double e = 0, last_e = 0, sum_e = 0;
double dt = 0.1;  // Thời gian mẫu
double de = 0;    // Đạo hàm của sai số

// Biến encoder
volatile long encoderCount = 0;
double pulsesPerRevolution = 360.0;  // Số xung encoder mỗi vòng quay

// Biến mạng RBF
const int NUM_NEURONS = 3;
double mu[NUM_NEURONS] = {0, 20, 100};  // Trung tâm neuron cho các giá trị sai số
double sigma[NUM_NEURONS] = {10, 15, 25};  // Độ rộng của hàm Gaussian
double weights[NUM_NEURONS][3];      // Trọng số cho Kp, Ki, Kd

void setup() {
  Serial.begin(9600);

  // Khởi tạo các chân động cơ và encoder
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
  
  // Khởi tạo trọng số cho mạng RBF
  initializeWeights();
}

void loop() {
  // Tính toán tốc độ động cơ từ encoder
  currentSpeed = calculateSpeed();

  // Tính sai số
  e = setpoint - currentSpeed;

  // In giá trị để giám sát
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" | Current Speed: ");
  Serial.print(currentSpeed);
  Serial.print(" | Kp: ");
  Serial.print(Kp);
  Serial.print(" | Ki: ");
  Serial.print(Ki);
  Serial.print(" | Kd: ");
  Serial.println(Kd);

  // Kiểm tra để tránh chia cho 0
  if (dt > 0) {
    de = (e - last_e) / dt;
  } else {
    de = 0;
  }

  // Tính toán tích phân và giới hạn tích phân
  double MAX_INTEGRAL = 1000;  // Giới hạn tích phân
  sum_e += e * dt;
  sum_e = constrain(sum_e, -MAX_INTEGRAL, MAX_INTEGRAL);  // Giới hạn sum_e

  // Tính toán tham số PID bằng mạng RBF
  computePIDRBF(e);

  // Điều khiển động cơ dựa trên tham số PID
  double controlSignal = Kp * e + Ki * sum_e + Kd * de;
  controlSignal = constrain(controlSignal, 0, 255);  // Giới hạn giá trị PWM từ 0 tới 255

  analogWrite(ENA, controlSignal);  // Xuất tín hiệu PWM điều khiển tốc độ động cơ

  // Điều khiển chiều quay động cơ
  if (controlSignal >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  last_e = e;

  delay(dt * 1000);  // Chu kỳ lặp
}

// Hàm ngắt encoder
void encoderISR() {
  encoderCount++;
}

// Hàm tính tốc độ từ encoder
double calculateSpeed() {
  static long lastEncoderCount = 0;
  static unsigned long lastTime = 0;

  // Tính toán thời gian và số xung đã quay được
  unsigned long currentTime = millis();
  long pulses = encoderCount - lastEncoderCount;

  // Tránh chia cho 0 khi tính tốc độ
  if (currentTime - lastTime > 0) {
    double speed = (pulses / pulsesPerRevolution) * (60000.0 / (currentTime - lastTime));  // RPM
    lastEncoderCount = encoderCount;
    lastTime = currentTime;
    return speed;
  } else {
    return 0;  // Nếu thời gian không thay đổi, tốc độ bằng 0
  }
}

// Khởi tạo các trọng số mạng RBF
void initializeWeights() {
  // Khởi tạo trọng số mạng RBF với các giá trị khác nhau cho từng neuron
  weights[0][0] = 2.0;  // Trọng số cho Kp từ neuron 1
  weights[0][1] = 1.5;  // Trọng số cho Ki từ neuron 1
  weights[0][2] = 0.5;  // Trọng số cho Kd từ neuron 1

  weights[1][0] = 1.5;  // Trọng số cho Kp từ neuron 2
  weights[1][1] = 0.9;  // Trọng số cho Ki từ neuron 2
  weights[1][2] = 0.4;  // Trọng số cho Kd từ neuron 2

  weights[2][0] = 1.0;  // Trọng số cho Kp từ neuron 3
  weights[2][1] = 0.5;  // Trọng số cho Ki từ neuron 3
  weights[2][2] = 0.3;  // Trọng số cho Kd từ neuron 3
}

// Tính toán giá trị PID từ mạng RBF
void computePIDRBF(double e) {
  double phi[NUM_NEURONS];
  double totalPhi = 0; // Tổng giá trị phi để chuẩn hóa

  for (int i = 0; i < NUM_NEURONS; i++) {
    phi[i] = exp(-pow(e - mu[i], 2) / (2 * pow(sigma[i], 2)));  // Hàm Gaussian
    totalPhi += phi[i];  // Cộng dồn để chuẩn hóa
    //Serial.print("phi["); Serial.print(i); Serial.print("]: "); Serial.println(phi[i]);  // In phi
  }
   //Serial.print("totalPhi: "); Serial.println(totalPhi);  // In totalPhi
  // Tính giá trị Kp, Ki, Kd từ mạng RBF, sử dụng tổng phi để chuẩn hóa
  Kp = 0;
  Ki = 0;
  Kd = 0;

  if (totalPhi > 0) { // Tránh chia cho 0
    for (int i = 0; i < NUM_NEURONS; i++) {
      Kp += (phi[i] / totalPhi) * weights[i][0];  // Trọng số Kp
      Ki += (phi[i] / totalPhi) * weights[i][1];  // Trọng số Ki
      Kd += (phi[i] / totalPhi) * weights[i][2];  // Trọng số Kd
    }
  }
}
