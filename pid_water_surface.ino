#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <pid.h>
#include <pidautotuner.h>
#include <string.h>

// #define PWM_INPUT_PIN 34
#define PWM_OUTPUT_PIN 12

/* Controller parameters */
#define DEFAULT_SETPOINT 13.0
#define DEFAULT_EPID_KP 31.14
#define DEFAULT_EPID_KI 0.000001
#define DEFAULT_EPID_KD 198633.73

/* Limit for PWM. */
#define PID_LIM_MIN 0.0 
#define PID_LIM_MAX 255.0

#define DEADBAND 0.0 /* Off==0 */

#define MSG_BUFFER_SIZE 50


bool tune = false;

epid_t pid_ctx;

float epid_kp = DEFAULT_EPID_KP;
float epid_ki = DEFAULT_EPID_KI;
float epid_kd = DEFAULT_EPID_KD;

float set_point = DEFAULT_SETPOINT;

//Define Variables we'll be connecting to
float pwm_input;
int pwm_output = 0;
float deadband_delta;

const int pingPin = 5;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 4;  // Echo Pin of Ultrasonic Sensor

long duration;

//---- WiFi settings
const char* ssid = "AndroidAPC861";
const char* password = "cpuq3193";

//---- MQTT Broker settings
const char* mqtt_server = "a58832d3b54d4cf39e7577eae6d3378e.s2.eu.hivemq.cloud";
const char* mqtt_username = "esp32client";
const char* mqtt_password = "esp32client";
const int mqtt_port = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;

long pid_time = 0;

float distance = 0;

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";



// pid controller setup
void setup_pid_controller() {
  /* Initialize serial and wait for port to open */
  // Serial.begin(9600);
  while (!Serial) { ; }

  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  // pinMode(PWM_INPUT_PIN, INPUT);
  analogWrite(PWM_OUTPUT_PIN, pwm_output);
  // pwm_input = analogRead(PWM_INPUT_PIN);
  pwm_input = distance;
  /* Initialize PID controller */
  epid_info_t epid_err = epid_init(&pid_ctx,
    pwm_input, pwm_input, PID_LIM_MIN,
    epid_kp, epid_ki, epid_kd
  );

  if (epid_err != EPID_ERR_NONE) {
    Serial.print("\n\n** ERROR: epid_err != EPID_ERR_NONE **\n\n");
    while (1) { ; }
  }
}

// pid controller loop
void loop_pid_controller() {
  // pwm_input = analogRead(PWM_INPUT_PIN);
  pwm_input = distance;
  pwm_input = set_point - (distance - set_point);

  epid_pid_calc(&pid_ctx, set_point, pwm_input); /* Calc PID terms values */

  /* Apply deadband filter to `delta[k]`. */
  deadband_delta = pid_ctx.p_term + pid_ctx.i_term + pid_ctx.d_term;
  if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= DEADBAND)) {  
    /* Compute new control signal output */
    epid_pid_sum(&pid_ctx, PID_LIM_MIN, PID_LIM_MAX);
    pwm_output = (int)lroundf(pid_ctx.y_out); /* float to int */
  }

  // pwm_output = 255 - pwm_output;
  // pwm_output = 0;
  // if (distance > set_point){
  //   analogWrite(PWM_OUTPUT_PIN, pwm_output);
  //   Serial.print("pwm_output = ");
  //   Serial.println(pwm_output);    
  // }
  // else{
  //   analogWrite(PWM_OUTPUT_PIN, 0);
  // }
  analogWrite(PWM_OUTPUT_PIN, pwm_output);
  Serial.print("pwm_input = ");
  Serial.print(pwm_input);
  Serial.print(", pwm_output = ");
  Serial.println(pwm_output);

  delay(500);
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output,. 
  // Changes the output state according to the message
  const char* msgtmp = messageTemp.c_str();
  if (String(topic) == "esp32/set") {
    set_point = atof(msgtmp);
    Serial.print("Changing Set Point to ");
    Serial.print(msgtmp);
    Serial.print(" cm.");
  }
  else if (String(topic) == "esp32/pid/kp") {
    epid_kp = atof(msgtmp);
    Serial.print("Changing kp to ");
    Serial.print(msgtmp);
    Serial.print(".");
  }
  else if (String(topic) == "esp32/pid/ki") {
    epid_ki = atof(msgtmp);
    Serial.print("Changing ki to ");
    Serial.print(msgtmp);
    Serial.print(".");
  }
  else if (String(topic) == "esp32/pid/kd") {
    epid_kd = atof(msgtmp);
    Serial.print("Changing kd to ");
    Serial.print(msgtmp);
    Serial.print(".");
  }
  else if (String(topic) == "esp32/tune") {
    if (atoi(msgtmp) == 1) {
      tune = true;
    }
    else {
      tune = false;
    }
    Serial.print("Changing tune to ");
    Serial.print(msgtmp);
    Serial.print(".");
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp32client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/set");
      client.subscribe("esp32/pid/kp");
      client.subscribe("esp32/pid/ki");
      client.subscribe("esp32/pid/kd");
      client.subscribe("esp32/tune");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void initSensor() {
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(pingPin, LOW);
  Serial.begin(9600);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void sensorLoop() {
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 29 / 2;
  Serial.print(distance);
  Serial.println(" cm");
  delay(300);
}

void setup_pid_autotuner(){
  PIDAutotuner tuner = PIDAutotuner();
  tuner.setTargetInputValue(set_point);
  tuner.setLoopInterval(500);
  tuner.setOutputRange(0, 255);
  tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);
  tuner.startTuningLoop(micros());
  long microseconds;
  while (!tuner.isFinished()){
    long prevMicroseconds = microseconds;
    microseconds = micros();
    
    sensorLoop();
    double input = set_point - (distance - set_point);
    double output = tuner.tunePID(input, microseconds);
    pwm_output = output;
    analogWrite(PWM_OUTPUT_PIN, pwm_output);
    Serial.print("input test = ");
    Serial.print(distance);
    Serial.print(", output test = ");
    Serial.println(pwm_output);
    while (micros() - microseconds < 500) delayMicroseconds(1);
  }

  pwm_output = 0;
  analogWrite(PWM_OUTPUT_PIN, pwm_output);

  double epid_kp = tuner.getKp();
  Serial.print("final kp = ");
  Serial.println(epid_kp);
  double epid_ki = tuner.getKi();
  Serial.print("final ki = ");
  Serial.println(epid_ki);
  double epid_kd = tuner.getKd();
  Serial.print("final kd = ");
  Serial.println(epid_kd);
}

void publishValueLoop() {
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Convert the value to a char array
    char tempString[8];
    dtostrf(distance, 1, 2, tempString);
    client.publish("esp32/value", tempString);
  }
}
char * buf;
void readSerialCOM() {
  // char *buf;
  // // Serial.readBytes(buf, 2);
  // // String bf1 = Serial.readStringUntil(',');
  // Serial.readBytesUntil(',', buf, 3);
  // Serial.flush();
  // // Serial.print("readStrUntil = ");
  // // Serial.println(bf1)
  // Serial.println(buf);
  char input[2];
  if (Serial.available() > 0) {
    // input[0] = (byte)Serial.read();
    // input[1] = '\0';

    // if (input[0] == ',') {
    //   Serial.println(buf);
    //   strcpy(buf, "");
    // }
    // else if ((int)input[0] >= 48 && (int)input[0] <= 57) {
    //   // char* tmp = "";
    //   strcat(buf, input);
    // }
    int t = Serial.parseInt();
    Serial.println(t);
    delay(500);
  }
}

void setup() {
  initSensor();
  Serial.begin(115200);
  initWiFi();

  espClient.setCACert(root_ca);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // setup_pid_autotuner();
  setup_pid_controller();
  // Serial.println();
  // Serial.println(ARDUINO);
}

void loop() {
  sensorLoop();
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  publishValueLoop();
  if (tune) {
    setup_pid_autotuner();
    tune = false;
  }
  long cur_time = micros();
  // if (cur_time - pid_time > 500) {
  //   pid_time = cur_time;
    loop_pid_controller();
  // }
  // readSerialCOM();
}
