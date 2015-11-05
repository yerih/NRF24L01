# NRF24L01
Radio project

#define BIT(x) (1<< (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS( (x) , ( BIT( (y) ) ) )
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y)))) 

#define CE_HIGH   PORTB |= (1<<PB1)
#define CE_LOW    PORTB &= ~(1<<PB1)

#define CSN_HIGH  PORTB |= (1<<PB2)
#define CSN_LOW   PORTB &= ~(1<<PB2)

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09

//Direcciones en memoria de los pipe
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10

#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     1
#define PRIM_TX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ENAA_ALL    63

#define ERX_P5      28
#define ERX_P4      44
#define ERX_P3      52
#define ERX_P2      56
#define ERX_P1      62
#define ERX_P0      1
#define ERX_ALL     3
#define toggleERX_ALL    252

#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define   W   1
#define   R   0

//Variables globales
uint8_t *enviar[1], *recibido;
volatile uint8_t int_boton, int_count0, int_count1;

void initSPI()
{
  //Set SCK(PB5), MOSI (PB3), CSN(SS & PB2) & CE as outport
  DDRB |= (1<<DDB5)|(1<<DDB3)|(1<<DDB2)|(1<<DDB1);
  //Habilita el SPI. Habilita el maestro. Fdiv = fclk/4
  SPCR |= (1<<SPE)|(1<<MSTR);
  SETBIT(PORTB, 2); //CSN IR_high
  CLEARBIT(PORTB, 1); //CE low to start with, nothing to send/receive yet!
    
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Ports Setup
  DDRB &= ~(1<<DDB0); //Pin B0 del Puerto B como entrada.
  DDRD = 0xff;        //Puerto D como salida.
  MCUCR &= ~(1<<PUD); //Habilita las resistencias de pull-up.

}

uint8_t WriteByteSPI(unsigned char cData)
{
  //Carga un byte en el registro de datos
  SPDR = cData;

  //Espera hasta que la transmision se complete
  while(!(SPSR & (1<<SPIF)));

  //Retorna lo que ha recibido del nRF
  return SPDR;
}

uint8_t RF_MOD_RX()
{
  uint8_t *var;
  var = WriteToNrf(R, CONFIG, var, 1); //Lee el registro CONFIG y almacena el resultado
  *var |= PRIM_RX; //Colocar en modo PRIM_RX (bit0 HIGH)
  
  WriteToNrf(W, CONFIG, var, 1); //Escribe en el registro config 

}

uint8_t RF_MOD_TX()
{
  uint8_t *var;
  var = WriteToNrf(R, CONFIG, var, 1); //Lee el registro CONFIG y almacena el resultado
  *var &= PRIM_TX; //Colocar en modo PRIM_TX (bit0 LOW)
  WriteToNrf(W, CONFIG, var, 1); //Escribe en el registro config 

}

void receive_payload(void)
{
  SETBIT(PORTB, 1); //CE IR_HIGH = leer datos.
  delayMicroseconds(50);
  CLEARBIT(PORTB, 1); //CE LOW - deja la lectura de datos.
}

void init_interrupt(){
  cli();                 // switch interrupts off while messing with their settings  
  EICRA = 0x0A;          // Falling edge. Interrupcion por Flancos de bajada.
  EIMSK = 0x00;          // Deshabilita interrupciones externas standard.
  PCICR = 0x03;          // Habilita las interrupciones 8 - 14 y 1 - 7.
  PCMSK0 = 0x01;         // Habilita unicamente la interrupcion 0.
  PCMSK1 = 0b00000100;   // Habilita unicamente la interrupcion 10.
  sei();    // turn interrupts back on
}

//Rutina de interrupcion externa (Boton de Llamada)
ISR(PCINT1_vect)
{ 
  /*Segun el manual del AVR, La interrupcion 
    por cambio de puerto se genera por cualquier
    cambio generado en el pin de interrupcion externa*/
  
  cli();                //desactiva interrupciones
  int_count1++;      
  if(int_count1 == 2)
  {
    int_boton = 1;      //Cambio de estado: reposo - llamada
    int_count1 = 0;      //reinicio de contador
    //PORTB ^= (1<<PB0);  //toggle de bit 0 de puerto B. 
  } 
  sei();                //Activa interrupciones
}

//Interrupcion para IRQ
ISR(PCINT0_vect)
{ 
  //Segun el manual del AVR, La interrupcion 
  //por cambio de puerto se genera por cualquier
  //cambio generado en el pin de interrupcion externa

  int_count0++;
  cli();                //desactiva interrupciones
    if(int_count0 == 2)
    {
       PORTD ^= (1<<PD7);
       int_count0 = 0;
       CLEARBIT(PORTB, 1);    //CE LOW - deja de leer o enviar.
        
       delayMicroseconds(10);
       recibido = WriteToNrf(R, R_RX_PAYLOAD, recibido, 1); //Lee el dato recibido
        
       reset();              // 
    }
  sei();                //Activa interrupciones
}

uint8_t GetReg(uint8_t reg)
{
  delayMicroseconds(10);
  CLEARBIT(PORTB, 2);   //CSN LOW
  delayMicroseconds(5);
  WriteByteSPI(R_REGISTER + reg);
  delayMicroseconds(10);
  reg = WriteByteSPI(NOP);
  delayMicroseconds(10);
  SETBIT(PORTB, 2);
  return reg;
}


void init_nRF24L01()
{
  delayMicroseconds(10);
  uint8_t val[5];  //Arreglo de enteros para ser enviados a la funcion *WriteToNrf
  
  //Habilitacion de auto ack para todos los pipes
  val[0] = 0x01;
  WriteToNrf(W, EN_AA, val, 1); //escribe en el registro EN_AA

  //Habilitacion de las pipes (modificable)
  //Habilita pipe 1 solamente
  val[0] = 0x01;
  WriteToNrf(W, EN_RXADDR, val, 1); //escribe en el registro 

 //Setup of Automatic Retransmission
  val[0] = 0x0F; //2ms de delay y cuenta deshabilitada                                                     
  WriteToNrf(W, SETUP_RETR, val, 1);
  
     //PARA LOS PRIMEROS 4bits MAS SIGNIFICATIVOS:
     //Auto retransmit delay
        //0000 250us
        //0001 500us
        //0010 750us
        //...
        //1111 4000us.

  //RF channel
    //channel 2.
  val[0] = 0x01;
  WriteToNrf(W, RF_CH, val, 1);

  //RF setup
    //PLL lock signal.
    //Air data rate 1Mbps
    //RF output power Tx mode 0dBm
    //Setup LNA gain
  val[0] = 0x26;
  WriteToNrf(W, RF_SETUP, val, 1);


  //Tamaño de address width
  val[0] = 0x03;  // bytes
  WriteToNrf(W, SETUP_AW, val, 1);

  //RX address para pipe del 1 al 6
    //address pipe 0
  int i;
  for(i=0; i<5; i++)  
  {
    val[i]=0x12;  //RF address
  }
  WriteToNrf(W, RX_ADDR_P0, val, 5);
  
  //Transmit address
  for(i=0; i<5; i++)  
  {
    val[i]=0x12;  
  }
  WriteToNrf(W, TX_ADDR, val, 5);

  //Numero de bytes en RX PAYLOAD de cada pipe
    //pipe 0
    val[0] = 0x05;
    WriteToNrf(W, RX_PW_P0, val, 1);

  //Habilitar tamaño de payload dinamico
/*  val[0] = 0xff;
    WriteToNrf(W, DYNPD, val ,1);
*/
  //Deshabilita payload dinamico
    //Habilita payload con ACK
    //Habilita comando NOACK para TX
    //val[0] = 0x04;
    //WriteToNrf(W, FEATURE, val, 1);

  //Configuracion de interrupciones y modos de operacion.
  val[0] = 0x3B;            
  WriteToNrf(W, CONFIG, val, 1);  //Interrupcion solo por RX. 
                                  //CRC habilitado. CRC esquema de codificacion de 1 byte. 
                                  //Encender. Operacion en Modo Rx.

  delay(2);     //tiempo de espera de estabilizacion
}

uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
  /* Esta funcion transfiere es capaz de leer y escribir en el nRF y deberia ser
        capaz de ejecutar un arreglo de enteros y retornar un arreglo de enteros*/
  if(ReadWrite == W) //Si es "W", significa que escribiras un registro 
  {
    reg = W_REGISTER + reg; 
  }

  //Crea un arreglo para ser retornado al final de la funcion
  static uint8_t ret[32];

  delayMicroseconds(10);
  CLEARBIT(PORTB, 2);     //CSN low - nRF empieza a escuchar por comandoo
  delayMicroseconds(10);
  WriteByteSPI(reg);      //setea el nRF para modo escritura o de lectura del registro "reg""
  delayMicroseconds(10);

  int i;
  for(i = 0; i< antVal; i++)
  {
    if(ReadWrite == R && reg != W_TX_PAYLOAD)  //Lectura de registro?
    {
      ret[i] = WriteByteSPI(NOP);   //envia comando "NOP" para leer los datos
      delayMicroseconds(10); 
    }

    else
    {
      WriteByteSPI(val[i]);     //envia el comando al nRF 
      delayMicroseconds(10);
    }
  }
  SETBIT(PORTB, 2);         //CSN High - nRF vuelve a su estado de reposo
  return ret;               //Retorna el arreglo
}

void transmit_payload(uint8_t *W_buff)
{
  WriteToNrf(R, FLUSH_TX, W_buff, 0);
    Serial.println(GetReg(STATUS));
    
    delay(100);
  WriteToNrf(R, W_TX_PAYLOAD, W_buff, 5);
      Serial.println(GetReg(STATUS));
    
    delay(100);
  //CSN_LOW;
  //WriteByteSPI(W_TX_PAYLOAD);
  //WriteByteSPI(*W_buff);
  //CSN_HIGH;  
  //delayMicroseconds(10);  //Tiempo de espera para que el nRF cargue el payload.
  delay(10);
  CE_HIGH;                  //CE high
  delayMicroseconds(20);   
  CE_LOW;     //CE low
  delay(10);
}


void reset(void)
{
  delayMicroseconds(5);
  CLEARBIT(PORTB, 2); //CSN LOW
  delayMicroseconds(5);
  WriteByteSPI(W_REGISTER + STATUS);  //Escribe en el registro STATUS
  delayMicroseconds(5);
  WriteByteSPI(0x70); //Resetea todos los IRQ del STTATUS register
  delayMicroseconds(5);
  SETBIT(PORTB, 2);   //CSN HIGH.
}

void transmit_init(void)
{
        delay(100);         // allow the radio to reach power-down if the shutdown
        uint8_t val[5];         // an array of integers that sends values ??to WriteToNrf function
       
        //EN_AA - (enable auto-acknowledgments) - Transmitter gets automatic response from receiver when successful transmission! (lovely function!)
        //Only works if Transmitter has identical RF_Adress on its channel ex: RX_ADDR_Po = TX_ADDR
        val[0]=0x01; //set value
        WriteToNrf(W, EN_AA, val, 1); //N=write mode, EN_AA=register to write to, val=data to write, 1=number of data bytes.
       
        //Choose number of enabled data pipes (1-5)
        val[0]=0x01;
        WriteToNrf(W, EN_RXADDR, val, 1); //enable data pipe 0
 
        //RF_Adress width setup (how many bytes is the receiver address, the more the merrier 1-5)
        val[0]=0x03; //0b0000 00011 = 5 bytes RF_Adress
        WriteToNrf(W, SETUP_AW, val, 1);
 
        // RF channel setup - select the frequency from 2.400 to 2.527 GHz 1MHz/steg
        val[0] = 0x01 ;
        WriteToNrf(W,RF_CH,val,1);  // RF channel registry 0b0000 0001 = 2.401 GHz (same on the TX RX)
 
        //RF setup - choose power mode and data speed. Here is the diference with the (+) version!!!
        val[0]=0x27; //00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode ("11" = -odB ; "00"=-18dB)
        WriteToNrf(W, RF_SETUP, val, 1);
 
        //RF_Adress setup 5 byte - Set Receiver address (set RX_ADDR_Po = TX_ADDR if EN_AA is enabled!!!)
        int i=0;
        for(i=0; i<5; i++){
                val[i]=0x11; //ox12 x 5 to get a long and secure address.
        }
        WriteToNrf(W, RX_ADDR_P0, val, 5); //since we chose pipe 0 on EN_RXADDR we give this address to that channel.
        //Here you can give different addresses to different channels (if they are enabled in EN_RXADDR) to listen on several different transmitters)
       
        //TX RF_Adress setup 5 byte - Set Transmitter address (not used in a receiver but can be set anyway)
        for(i=0; i<5; i++){
                val[i]=0x11; //ox12 x 5 - same on the Receiver chip and the RX-RF_Address above if EN_AA is enabled!!!
        }
        WriteToNrf(W, TX_ADDR, val, 5);
 
        //Payload width Setup - 1-32byte (how many bytes to send per transmission)
        val[0]=0x05; //Send 5 bytes per package this time (same on receiver and transmitter)
        WriteToNrf(W,RX_PW_P0,val,1);
 
        val[0]=0x2F; //0b00l0 00011 "2" sets it up to 7SouS delay between every retry (at least Seeus at 25okbps and if payload >5bytes in 1Hbps,
        //and if payload >1Sbyte in 2Hbps) "F" is number of retries (1-15, now 15)
        WriteToNrf(W, SETUP_RETR, val, 1);
 
        //CONFIG reg setup - Now it's time to boot up the Qgf and choose if it's suppose to be a transmitter or receiver
        val[0]=0x1E; //0b0001 1110 - bit 0="0":transmitter bit 0="1":Receiver, bit 1="1"=power up,
        //bit 4="1"= mask_Max_RT i.e. IRQ-interrupt is not triggered if transmission failed.
        WriteToNrf(W, CONFIG, val, 1);
 
        //device need 1.5ms to reach standby mode (CE=low)
        delay(100);
}

void send_data(uint8_t * tx_payload)
{
  
  WriteToNrf(R,FLUSH_TX,tx_payload,0);
  WriteToNrf(R,W_TX_PAYLOAD,tx_payload,3);//Load Payload of length 5  

  delayMicroseconds(10);
  CE_HIGH;//Start Transmitting
  delayMicroseconds(20);
  CE_LOW;//Stop transmitting
  delayMicroseconds(10);
}

void loop() 
{

  //Inicializacion de perifericos
  initSPI();
  transmit_init();
  //init_nRF24L01();
  //init_interrupt();
  
  uint8_t w_buf[5];
  for(int i=0;i<3;i++)
  {
    w_buf[i]=0x41;
  }
  
  while(1)
  {

    //send_data(w_buf);
    transmit_payload(w_buf);
    reset();
    Serial.println(GetReg(STATUS));
    delay(100);
    delayMicroseconds(10);
  }  
   
}
