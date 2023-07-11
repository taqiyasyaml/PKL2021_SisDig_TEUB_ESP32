#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoWebsockets.h>

#define IN_7_C_PIN 2

#define O_NOT_I_PIN 4
#define READ_EN_PIN 13

#define D_PIN 17
#define A0_PIN 18
#define A1_PIN 19
#define A2_PIN 21

#define SET_IN_ADDR_PIN 23
#define SET_OUT_ADDR_PIN 22
#define RESET_OUT_ADDR_PIN 16

#define ADC_A0 34
#define ADC_A1 35
#define ADC_A2 32
#define ADC_A3 33
#define ADC_A4 25
#define ADC_A5 26
#define ADC_A6 27
#define ADC_A7 14

#define PROPAGATION_DELAY_MS 2

using namespace websockets;
WebsocketsClient ws_client;
void wsOnMessageCallback(WebsocketsMessage message);
void wsOnEventsCallback(WebsocketsEvent event, String data) ;
long last_try_ws = 0;
int ws_state = 4;

bool is_host = false;

bool out_in[256][256];

long delay_read_minimum = 500;
long last_send_read=0;
float last_read_io[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float in_out_adc_margin[2] = {-0.05,-0.05};
float in_out_adc[2][8];
bool in_7_no = false;

void write_control(int pin);

void ws_check();
void serial_json_check();
bool read_check();
void read_loop();
void read_io(int type);
bool read_io_is_change(int type, float new_adc[8]);

bool set_addr(int addr);
bool set_output_one_input(int out, int in);
bool write_output_one(int out, int in, bool val, bool save);
bool write_output(int out, int in);
bool sync_output(int out);
void sync_output_all();
void sync_in_7_c();
bool reset_ouput(int out, bool save);
void reset_output_all();

DynamicJsonDocument last_json(24576);
void parse_json();
bool recv_sync();
bool recv_set_write();
bool recv_set_read();
bool recv_reset_out();
bool recv_set_in_7();
bool recv_read_io();

void send_set_write();
void send_set_read(bool force_read_in, bool force_read_out);

WebServer http_server(80);

bool get_conf_spiffs();
long last_0_pressed=0;
void check_and_wifi_mode();
void switch_wifi();
void connect_wifi();
void switch_ap();

void setup_http_server();
void srv_send_file_200_handler();
void srv_send_html(int status_code, String f);
void post_index_handler();

String ssid = "";
String password = "";
String ws_url = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IN_7_C_PIN,OUTPUT);
  pinMode(D_PIN,OUTPUT);
  pinMode(A0_PIN ,OUTPUT);
  pinMode(A1_PIN ,OUTPUT);
  pinMode(A2_PIN,OUTPUT);
  pinMode(SET_IN_ADDR_PIN ,OUTPUT);
  pinMode(SET_OUT_ADDR_PIN ,OUTPUT);
  pinMode(RESET_OUT_ADDR_PIN ,OUTPUT);
  pinMode(O_NOT_I_PIN, OUTPUT);
  pinMode(READ_EN_PIN, OUTPUT);

  pinMode(ADC_A7, INPUT);
  pinMode(ADC_A6, INPUT);
  pinMode(ADC_A5, INPUT);
  pinMode(ADC_A4, INPUT);
  
  for(int o = 0; o<8; o++){
    for(int i = 0; i<8; i++)
      out_in[o][i] = false;
  }
  in_7_no=false;
  sync_output_all();
  for(int addr=0;addr<8;addr++){
    in_out_adc[0][addr]=0.0;
    in_out_adc[1][addr]=0.0;
  }
  ws_client.onMessage(wsOnMessageCallback);
  ws_client.onEvent(wsOnEventsCallback);
    if(!SPIFFS.begin(true)){
     Serial.println("SPIFFS Mount Failed");
     return;
  }
  check_and_wifi_mode();
  setup_http_server();
  pinMode(0,INPUT);
  send_set_read(false,false);
  send_set_write();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(READ_EN_PIN,LOW);
  serial_json_check();
  check_and_wifi_mode();
  read_loop();
  if(ws_state==1)
    ws_client.poll();
  http_server.handleClient();
}

void serial_json_check(){
  if(Serial.available()){
    DeserializationError err = deserializeJson(last_json,Serial);
    if(!err)
      parse_json();    
  }
}

void check_and_wifi_mode(){
    if(digitalRead(0)==LOW){
      if(last_0_pressed==0)
        last_0_pressed=millis();
      else if(millis()-last_0_pressed>=3000){
        if(is_host)
          switch_wifi();
        else
          switch_ap();
        in_7_no = !in_7_no;
        sync_in_7_c();
        while(digitalRead(0)==LOW) delay(1);
        in_7_no = !in_7_no;
        sync_in_7_c();
        last_0_pressed=0;
      }
    }else if(last_0_pressed>0)
      last_0_pressed=0;
    else{
      if(!is_host&&WiFi.status()!=WL_CONNECTED)
        switch_wifi();
      else if(WiFi.status()==WL_CONNECTED&&ws_state!=1)
        check_ws();
    }
}

void check_ws(){
  if(WiFi.status() == WL_CONNECTED && ws_state > 1){
    Serial.println("Try connect ws ("+ws_url+")");
    ws_client.connect(ws_url.c_str());
  }
}

void read_loop(){
  if(delay_read_minimum >= 0 && (millis()-last_send_read)>=delay_read_minimum && read_check())
   send_set_read(false,false);
}

bool read_check(){
  bool is_change = false;
  for(int type=0;type<2;type++){
    if(in_out_adc_margin[type] >= 0.0){
      read_io(type);
      is_change = read_io_is_change(type,in_out_adc_margin[type],last_read_io) || is_change;
    }else{
      float read_adc[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      is_change = read_io_is_change(type,0.01,read_adc) || is_change;
    }
  }
  return is_change;
}

void read_io(int type){
  digitalWrite(O_NOT_I_PIN,type==0?LOW:HIGH);
  digitalWrite(READ_EN_PIN,HIGH);
  delay(PROPAGATION_DELAY_MS);
  last_read_io[7] = digitalRead(ADC_A7)*5.0;
  last_read_io[6] = digitalRead(ADC_A6)*5.0; 
  last_read_io[5] = digitalRead(ADC_A5)*5.0;
  last_read_io[4] = digitalRead(ADC_A4)*5.0;
  last_read_io[3] = (analogRead(ADC_A3)/4095.0)*6.0;
  last_read_io[2] = (analogRead(ADC_A2)/4095.0)*6.0;
  last_read_io[1] = (analogRead(ADC_A1)/4095.0)*6.0;
  last_read_io[0] = (analogRead(ADC_A0)/4095.0)*6.0;
  digitalWrite(O_NOT_I_PIN,LOW);
  digitalWrite(READ_EN_PIN,LOW);
}

bool read_io_is_change(int type, float margin_io, float new_adc[8]){
  bool is_change = false;
  float margin_adc = 0.0;
  bool need_change = margin_io>=0.0 && margin_io<=5.0;
  for(int addr=0;addr<8;addr++){
    if(need_change){
      margin_adc=new_adc[addr]<in_out_adc[type][addr]?in_out_adc[type][addr]-new_adc[addr]:new_adc[addr]-in_out_adc[type][addr];
      if(margin_adc >= margin_io){
        is_change = true;
        need_change = false;
      }
    }
    in_out_adc[type][addr]=new_adc[addr];
  }
  return is_change;
}


void write_control(int pin){
  digitalWrite(pin,HIGH);
  delay(PROPAGATION_DELAY_MS);
  digitalWrite(pin,LOW);
}

bool set_addr(int addr){
  switch(addr){
    case 0:
      digitalWrite(A2_PIN,LOW);
      digitalWrite(A1_PIN,LOW);
      digitalWrite(A0_PIN,LOW);
      break;
    case 1:
      digitalWrite(A2_PIN,LOW);
      digitalWrite(A1_PIN,LOW);
      digitalWrite(A0_PIN,HIGH);
      break;
    case 2:
      digitalWrite(A2_PIN,LOW);
      digitalWrite(A1_PIN,HIGH);
      digitalWrite(A0_PIN,LOW);
      break;
    case 3:
      digitalWrite(A2_PIN,LOW);
      digitalWrite(A1_PIN,HIGH);
      digitalWrite(A0_PIN,HIGH);
      break;
    case 4:
      digitalWrite(A2_PIN,HIGH);
      digitalWrite(A1_PIN,LOW);
      digitalWrite(A0_PIN,LOW);
      break;
    case 5:
      digitalWrite(A2_PIN,HIGH);
      digitalWrite(A1_PIN,LOW);
      digitalWrite(A0_PIN,HIGH);
      break;
    case 6:
      digitalWrite(A2_PIN,HIGH);
      digitalWrite(A1_PIN,HIGH);
      digitalWrite(A0_PIN,LOW);
      break;
    case 7:
      digitalWrite(A2_PIN,HIGH);
      digitalWrite(A1_PIN,HIGH);
      digitalWrite(A0_PIN,HIGH);
      break;
    default:
      digitalWrite(A2_PIN,LOW);
      digitalWrite(A1_PIN,LOW);
      digitalWrite(A0_PIN,LOW);
      return false;
      break;
  }
  return true;
}

bool set_output_one_input(int out, int in){
  if(out < 0 || in < 0 || out > 7 || in > 7)
    return false;
  reset_output(out,true);
  out_in[out][in]=true;
  return sync_output(out);
}

bool write_output_one(int out, int in, bool val, bool save){
  if(!set_addr(in))
    return false;
  write_control(SET_IN_ADDR_PIN);
  if(!set_addr(out))
    return false;
  digitalWrite(D_PIN,val==true?HIGH:LOW);
  write_control(SET_OUT_ADDR_PIN);
  digitalWrite(D_PIN,LOW);
  set_addr(0);
  if(save==true)
    out_in[out][in] = val==true;
  return true;
}

bool write_output(int out, int in){
  if(out < 0 || in < 0 || out > 7 || in > 7)
    return false;
  return write_output_one(out,in,out_in[out][in],false);
}

bool sync_output(int out){
  if(out<0 || out>7)
    return false;
  reset_output(out,false);
  for(int in = 0; in<8; in++)
    write_output(out,in);
  return true;
}

void sync_output_all(){
  for(int out = 0; out<8; out++)
    sync_output(out);
  sync_in_7_c();
}

void sync_in_7_c(){
  digitalWrite(IN_7_C_PIN, in_7_no == true?HIGH:LOW);
  delay(PROPAGATION_DELAY_MS);
}

bool reset_output(int out, bool save){
  if(!set_addr(out))
    return false;
  write_control(RESET_OUT_ADDR_PIN);
  set_addr(0);
  if(save){
    for(int in=0;in<8;in++)
      out_in[out][in]=false;
  }
  return true;
}

void reset_output_all() {
  for(int out = 0; out<8; out++)
    reset_output(out,true);
}

void parse_json(){
  if(recv_sync())
    Serial.println("Receive Sync");
  else if (recv_set_write())
    Serial.println("Receive Set Write");
  else if (recv_set_read())
    Serial.println("Receive Set Read");
  else if (recv_reset_out())
    Serial.println("Receive Reset Out");
  else if (recv_set_in_7())
    Serial.println("Receive Set In 7");
  else if (recv_read_io())
    Serial.println("Receive Read IO");
  else{
  Serial.println("Unknown Req");
  serializeJson(last_json,Serial);
  }
}

bool recv_sync(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("sync"))
    return false;
  sync_output_all();
  read_check();
  send_set_write();
  send_set_read(false,false);
  return true;
}
bool recv_reset_out(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("reset_out"))
    return false;
  JsonVariant temp;
  temp=last_json["all"];
  if(!temp.isNull() && temp.as<bool>() == true)
    reset_output_all();
  else{
    for(int o=0;o<8;o++){
      temp=last_json["o_"+((String)o)];
        if(!temp.isNull() && temp.as<bool>() == true)
          reset_output(o,true);
    }
  }
  send_set_write();
  return true;
}
bool recv_set_write(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("set_write"))
    return false;
  JsonVariant temp;
  for(int o=0;o<8;o++){
    for(int i=0;i<8;i++){
      temp=last_json["o_"+((String)o)]["i_"+((String)i)]; 
      if(!temp.isNull())
        write_output_one(o,i,temp.as<bool>(),true);
    }
  }
  temp=last_json["i_7_no"];
  if(!temp.isNull()){
    in_7_no = temp.as<bool>();
    sync_in_7_c();
  }
  send_set_write();
  return true;
}
bool recv_set_read(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("set_read"))
    return false;
  JsonVariant temp;
  temp=last_json["i_margin"];
  if(!temp.isNull())
    in_out_adc_margin[0]=temp.as<float>();
  temp=last_json["o_margin"];
  if(!temp.isNull())
    in_out_adc_margin[1]=temp.as<float>();
  temp=last_json["dly_r_min_ms"];
  if(!temp.isNull())
    delay_read_minimum=temp.as<long>();
  read_check();
  send_set_read(false,false);
  return true;
}
bool recv_set_in_7(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("set_in_7"))
    return false;
  JsonVariant temp;
  temp=last_json["i_7_no"];
  if(!temp.isNull()){
    bool curr_i_7 = in_7_no;
    in_7_no = temp.as<bool>();
    if(in_7_no != curr_i_7)
      sync_in_7_c();
  }
  int us = 0;
  int ms = 0;
  temp=last_json["dly_us"];
  if(!temp.isNull())
    us = temp.as<int>();
  temp=last_json["dly_ms"];
  if(!temp.isNull())
    ms = temp.as<int>();
  digitalWrite(IN_7_C_PIN,in_7_no==false?HIGH:LOW);
  if(us>0)
    delayMicroseconds(us);
  if(ms>0)
    delay(ms);
  digitalWrite(IN_7_C_PIN,in_7_no==true?HIGH:LOW);
  send_set_write();
  return true;
}
bool recv_read_io(){
  if(!last_json["req"].as<String>().equalsIgnoreCase("read_io"))
    return false;
  bool force_read_io[2]={false,false};
  JsonVariant temp;
  temp=last_json["i"];
  if(!temp.isNull())
    force_read_io[0] = temp.as<bool>();  
  temp=last_json["o"];
  if(!temp.isNull())
    force_read_io[1] = temp.as<bool>();
  send_set_read(false,false);
  return true;
}

void send_set_write(){
  StaticJsonDocument<1600> send_json;
  send_json["req"]="set_write";
  send_json["esp_time"]=millis();
  for(int out=0;out<8;out++){
    for(int in=0;in<8;in++)
      send_json["o_"+((String)out)]["i_"+((String)in)] = out_in[out][in]; 
  }
  send_json["i_7_no"]=in_7_no;
  String ret;
  serializeJson(send_json,ret);
  Serial.println(ret);
  if(ws_state==1)
    ws_client.send(ret);
}

void send_set_read(bool force_read_in, bool force_read_out){
  last_send_read=millis();
  StaticJsonDocument<400> send_json;
  send_json["req"]="set_read";
  send_json["esp_time"]=millis();
  send_json["i_margin"]=in_out_adc_margin[0];
  send_json["o_margin"]=in_out_adc_margin[1];
  send_json["dly_r_min_ms"]=delay_read_minimum;
  float send_in_out_adc[2][8];
  for(int addr=0;addr<8;addr++){
    send_in_out_adc[0][addr]=in_out_adc[0][addr];
    send_in_out_adc[1][addr]=in_out_adc[1][addr];
  }
  if(force_read_in==true){
    read_io(0);
    for(int addr=0;addr<8;addr++)
      send_in_out_adc[0][addr]=last_read_io[addr];
  }
  if(force_read_in==true){
    read_io(0);
    for(int addr=0;addr<8;addr++)
      send_in_out_adc[1][addr]=last_read_io[addr];
  }
  for(int addr=0;addr<8;addr++){
    send_json["i_"+((String)addr)]=send_in_out_adc[0][addr];
    send_json["o_"+((String)addr)]=send_in_out_adc[1][addr];
  }
  String ret;
  serializeJson(send_json,ret);
  Serial.println(ret);
  if(ws_state==1)
    ws_client.send(ret);
}

bool get_conf_spiffs(){
  ssid = "";
  password = "";
  ws_url = "";
  DynamicJsonDocument conf_json(350);
  File file = SPIFFS.open("/config.json", "r");
  DeserializationError error = deserializeJson(conf_json, file);
  file.close();
  if (!error) {
    JsonVariant j_d = conf_json["ssid"];
    ssid = j_d.isNull()?"":j_d.as<String>();
    j_d = conf_json["password"];
    password = j_d.isNull()?"":j_d.as<String>();
    j_d = conf_json["ws"];
    ws_url = j_d.isNull()?"":j_d.as<String>();
  }
  return !ssid.equals("") && !ws_url.equals("");
}

void switch_wifi(){
  if(is_host){
    Serial.println("AP Turned Off");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    is_host=false;
  }else if(WiFi.status()==WL_CONNECTED){
    if(ws_state==1){
      ws_client.close();
      delay(PROPAGATION_DELAY_MS);
    }
    WiFi.disconnect();
    ws_state = 4;
  }
  if(get_conf_spiffs() && !ssid.equals("")){
    if(password.equals("")){
      WiFi.begin(ssid.c_str());
    }else{
      WiFi.begin(ssid.c_str(),password.c_str());
    }
    const bool curr_i_7_no = in_7_no;
    long startmillis = millis();
    int beforeDelaySwitch = digitalRead(0);
    long last_blink = 0;
    while(WiFi.status()!=WL_CONNECTED && (digitalRead(0)==LOW || millis()-startmillis<10000) && digitalRead(0)==beforeDelaySwitch){
      if((millis()-last_blink)>750){
        last_blink = millis();
        in_7_no = !in_7_no;
        sync_in_7_c();
      }
      delay(1);
    }
    in_7_no = curr_i_7_no;
    sync_in_7_c();
    if(WiFi.status()==WL_CONNECTED){
      is_host=false;
      Serial.println(WiFi.localIP());
      check_ws();
    }else
      Serial.println("Gagal Terhubung");
  }else if(!is_host)
    switch_ap();
}

void switch_ap(){
  if(!is_host || WiFi.status() == WL_NO_SHIELD){
    if(WiFi.status()==WL_CONNECTED){
       if(ws_state==1){
        ws_client.close();
        delay(PROPAGATION_DELAY_MS);
      }
      WiFi.disconnect();
    }
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ModulMatrix");
    Serial.println("Nama SSID : ModulMatrix");
    delay(100);
    IPAddress myIP = WiFi.softAPIP();
    Serial.println(myIP);
    is_host=true;
  }
}

void setup_http_server(){
  http_server.on("/",HTTP_POST,post_index_handler);
  http_server.on("/index.html",HTTP_POST,post_index_handler);
  http_server.onNotFound(srv_send_file_200_handler);
  http_server.begin();
}

void srv_send_file_200_handler(){
  String f = http_server.uri();
  f.toLowerCase();
  Serial.println(f);
  if(f.equals("/404.html")||f.equals("/post_success.html")||f.equals("/post_failed.html")){
    srv_send_html(404,"/404.html");
    return;
  }
  String dataType = "text/plain";
  if (f.endsWith(".css")) {
    dataType = "text/css";
  } else if (f.endsWith(".js")) {
    dataType = "application/javascript";
  }else if (f.endsWith(".html")) {
    dataType="text/html";
  }
  File dataFile = SPIFFS.open(f.c_str(),"r");
  if(dataFile.isDirectory()){
    dataFile.close();
    if(!f.endsWith("/"))
      f+="/";
    f+="index.html";
    dataType="text/html";
    dataFile = SPIFFS.open(f.c_str(),"r");
  }
  if (!SPIFFS.exists(f.c_str()) || !dataFile ) {
    dataFile.close();
    srv_send_html(404,"/404.html");
    return;
  }
  http_server.streamFile(dataFile, dataType);
  dataFile.close();
}

void srv_send_html(int status_code,String f){
  f.toLowerCase();
  if (!f.endsWith(".html")) {
    status_code=404;
    f = "/404.html";
  }
  if(!SPIFFS.exists(f.c_str())){
    status_code=404;
    f = "/404.html";
  }
  File dataFile = SPIFFS.open(f.c_str());
  if (!dataFile) {
    dataFile.close();
    dataFile = SPIFFS.open("/404.html");
    status_code=404;
  }
  String html = "";
  Serial.println();
  while(dataFile.available()){
    char d = dataFile.read();
    html+=(String)d;
    Serial.print(d);
  }
  Serial.println();
  dataFile.close();
  http_server.send(status_code,"text/html",html);
}

void post_index_handler(){
  get_conf_spiffs();
  if(http_server.arg("ssid")!="")
    ssid = http_server.arg("ssid");
  if(http_server.arg("password")!="")
    password = http_server.arg("password");
  if(http_server.arg("ws")!="")
    ws_url = http_server.arg("ws");
  if(ssid!=""&&ws_url!=""){
    StaticJsonDocument<400> js;
    js["ssid"]=ssid;
    js["password"]=password;
    js["ws"]=ws_url;
    String js_str;
    serializeJson(js,js_str);
    File dataFile=SPIFFS.open("/config.json",FILE_WRITE);
    if(dataFile){
      if(dataFile.print(js_str.c_str())){
        dataFile.close();
        srv_send_html(200,"/post_success.html");
        delay(5000);
        switch_wifi();
        return;
      }
    }
    dataFile.close();
    srv_send_html(500,"/post_failed.html");
    return;
  }
  srv_send_html(400,"/post_failed.html");
}

void wsOnMessageCallback(WebsocketsMessage message) {
  DeserializationError err = deserializeJson(last_json,message.data());
  if(!err)
    parse_json();
}

void wsOnEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("WS Connected");
        ws_state = 1;
        last_try_ws=0;
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("WS DisConnected");
        ws_state = 4;
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("WS Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.println("WS Got a Pong!");
    }
}
