/*
 * homeclient.cpp
 *
 * Created: 23-10-2013 22:44:49
 *  Author: LECAS
 */ 

#include "Arduino.h"
#include "RF24/SPI/SPI.h"
#include "RF24/nRF24L01.h"
#include "RF24/RF24.h"
#include "avr/eeprom.h"
#include "printf.h"
#include "avr/wdt.h"

#define ADDRconfig			0x01
#define SERVERpipe			0x02
#define configMODE			0x03
#define pingpong			0x04
#define searchMODE			0x05
#define set_server_route	0x06
#define found_dev			0x07
#define configured			0xAF
#define add_address			0x08
#define scan_zone			0x09
#define not_found			0x10
#define destroy_net			0x11
#define switch_event		0x12
#define all_on				0x13
#define all_off				0x14
#define output_toogle		0x15
#define set_active			0x16

struct {
	uint64_t destino;
	byte comando;
	byte dados[8];
}RFbuffer;

struct wifinetwork{
	uint64_t addr1;
	uint64_t addr2;
	uint8_t zone;
	}addresses;

/*eeprom vars*/
uint8_t		EEMEM	configFLAG;
uint64_t	EEMEM	server_pipe_addr;
wifinetwork	EEMEM	networkconfigurations;

/*inputs and outputs stuff*/
uint8_t outputs[3]={3,4,5};
uint8_t	last_input_states[4]={NULL};
uint8_t first_read=NULL;

/*wifi stuff*/
RF24 radio(10,9);
uint64_t server_pipe=NULL;
uint64_t mask=0xFFFFFFFFFF000000LL;

long currentTime=0;

void wait_for_config(){
	uint64_t addr=0;
	//espera por uma ligação
	while(!radio.available());
	//le os dados e cria os pipes
	radio.read(&RFbuffer,sizeof(RFbuffer));
	printf("received server addr:%lX%08lX\r\n",(uint32_t)(RFbuffer.destino>>32),(uint32_t)(RFbuffer.destino));
	addr=RFbuffer.destino;
	addr=(addr>>(32-(RFbuffer.dados[0]*8)));
	addr|=(byte)random(1,254);
	addresses.addr1=(addr<<(32-(RFbuffer.dados[0]*8)));
	addresses.addr2=(addr<<(32-(RFbuffer.dados[0]*8)));
	addresses.addr2|=0xFF;
	addresses.zone=RFbuffer.dados[0];
	printf("forward pipe addr:%lX%08lX, rewind pipe addr:%lX%08lX \r\n",(uint32_t)(addresses.addr1>>32),(uint32_t)(addresses.addr1),(uint32_t)(addresses.addr2>>32),(uint32_t)(addresses.addr2));
	if (RFbuffer.dados[0]==1){
		server_pipe=RFbuffer.destino;
	}else{
		server_pipe=RFbuffer.destino|0xFF;
	}
	printf("server addr:%lX%08lX\r\n",(uint32_t)(server_pipe>>32),(uint32_t)server_pipe);
	//guarda os endereços gerados e coloca-se em modo configurado
	eeprom_write_block(&addresses,&networkconfigurations,sizeof(networkconfigurations));
	eeprom_write_block(&server_pipe,&server_pipe_addr,sizeof(server_pipe_addr));
	eeprom_write_byte(&configFLAG,configured);
	delay(150);
	//envia o seu endereço para a zona 1(servidor)
	RFbuffer.destino=addresses.addr1;
	radio.stopListening();
	radio.openWritingPipe(server_pipe);
	boolean ok=radio.write(&RFbuffer,sizeof(RFbuffer));
	if (ok){
		printf("sent addr\r\n");
	}
	radio.startListening();
	wdt_enable(WDTO_500MS);
	while (1);
	
}
void output_writer(uint8_t index,uint8_t valor){
	digitalWrite(outputs[index],valor);
}
int temperature(){
	
	long val=0;
	for (int i=0;i<10;i++){
		val+=analogRead(3);
	}
	val/=10;
	float temp=((593-val)/1.94);
	return round(temp);
	
}
void message_handler(){
	boolean ok;
	switch (RFbuffer.comando){
		case scan_zone:
			radio.stopListening();
			radio.openWritingPipe((uint64_t)RFbuffer.dados[1]);
			ok=radio.write(&RFbuffer,sizeof(RFbuffer));
			radio.startListening();
			if (!ok){
				delay(50);
				RFbuffer.comando=not_found;
				radio.stopListening();
				radio.openWritingPipe(server_pipe);
				radio.write(&RFbuffer,sizeof(RFbuffer));
				radio.startListening();
			}
			break;
		case destroy_net:
			eeprom_write_byte(&configFLAG,0);
			wdt_enable(WDTO_500MS);
			while(1);
			break;
		case all_on:
			for (int i=0;i<sizeof(outputs);i++){
				digitalWrite(outputs[i],HIGH);
			}
			break;
		case all_off:
			for (int i=0;i<sizeof(outputs);i++){
				digitalWrite(outputs[i],LOW);
			}
			break;
		case output_toogle:
			output_writer(RFbuffer.dados[0],RFbuffer.dados[1]);
			break;
		case set_active:
			break;
	}
}
void RF24handler(){
	
	uint8_t pipe_number;
	if (radio.available(&pipe_number)){

		if (pipe_number==1){
			radio.read(&RFbuffer,sizeof(RFbuffer));
			if (RFbuffer.destino==addresses.addr1){
				message_handler();
			}else{
				radio.stopListening();
				radio.openWritingPipe((RFbuffer.destino&(mask>>(8*addresses.zone))));
				radio.write(&RFbuffer,sizeof(RFbuffer));
				radio.startListening();
			}
		}
		if (pipe_number==2){
			radio.read(&RFbuffer,sizeof(RFbuffer));
			radio.stopListening();
			radio.openWritingPipe(server_pipe);
			radio.write(&RFbuffer,sizeof(RFbuffer));
			radio.startListening();
			
		}		
	}
}
void input_reader(){
	uint8_t ins[3];
	uint16_t reading;
	for (int i=0;i<3;i++){
		for (int n=0;n<5;n++){
			reading+=analogRead(i);
			delay(1);
		}
		reading/=5;
		if (reading>950){
			ins[i]=HIGH;
		}else{
			ins[i]=LOW;
		}
	}
	for (int i=0;i<4;i++){
		if (ins[i]!=last_input_states[i]){
			last_input_states[i]=ins[i];
			if (first_read==1){
				RFbuffer.destino=addresses.addr1;
				RFbuffer.comando=switch_event;
				RFbuffer.dados[0]=i;
				radio.stopListening();
				radio.openWritingPipe(server_pipe);
				radio.write(&RFbuffer,sizeof(RFbuffer));
				radio.startListening();
			}
		}
	}
	first_read=1;
}
void setup(){
	wdt_disable();
	Serial.begin(57600);
	printf_begin();
	randomSeed(analogRead(7));
	for (int i=0;i<sizeof(outputs);i++){
		pinMode(outputs[i],OUTPUT);
	}
	radio.begin();
	radio.setRetries(20,40);
	radio.setPayloadSize(sizeof(RFbuffer));
	radio.setPALevel(RF24_PA_MAX);

	if (eeprom_read_byte(&configFLAG)==configured){
		eeprom_read_block(&addresses,&networkconfigurations,sizeof(networkconfigurations));
		eeprom_read_block(&server_pipe,&server_pipe_addr,sizeof(server_pipe_addr));
		radio.openReadingPipe(1,addresses.addr1);
		radio.openReadingPipe(2,addresses.addr2);
		radio.startListening();
	}else{
		uint64_t default_reading_pipe=(uint8_t)random(1,255);
		radio.openReadingPipe(1,default_reading_pipe);
		radio.startListening();
		printf("waiting for config...\r\n");
		wait_for_config();
	}
	analogReference(INTERNAL);
	delay(100);
}
void loop(){
	
	RF24handler();
	input_reader();
	if (millis()-currentTime>5000){
		currentTime=millis();
		Serial.print(temperature());
		Serial.println("ºC");
	}
}