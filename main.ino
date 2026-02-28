/**
 * PROJETO FINAL: Controle de Vazão e Nível de Tanque (Baremetal ATmega328P)
 * Descrição: Sistema de controle PID para vazão com monitoramento de nível via Ultrassom.
 * Interface: LCD 16x2 (I2C), Encoder Rotativo para ajuste de ganhos e Telemetria UART.
 */

typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;

// Macros para acesso direto aos registradores de memória (Endereços do Datasheet)
#define REG8(x)  (*(volatile uint8_t *)(x))
#define REG16(x) (*(volatile uint16_t *)(x))

// --- REGISTRADORES DE I/O E SISTEMA ---
#define _PIND   REG8(0x29) // Leitura dos pinos do Pórtico D
#define _DDRD   REG8(0x2A) // Direção dos pinos do Pórtico D
#define _PORTD  REG8(0x2B) // Escrita/Pull-up dos pinos do Pórtico D
#define _PINB   REG8(0x23) 
#define _DDRB   REG8(0x24) 
#define _PORTB  REG8(0x25) 
#define _SREG   REG8(0x5F) // Status Register (Controle de Interrupções Globais)
#define _EICRA  REG8(0x69) // Configuração de sentido das interrupções externas
#define _EIMSK  REG8(0x3D) // Máscara de habilitação de interrupções externas

// --- REGISTRADORES TIMER 1 (16 BITS) - Controle do Servo ---
#define _TCCR1A REG8(0x80) // Timer/Counter Control Register A
#define _TCCR1B REG8(0x81) // Timer/Counter Control Register B
#define _ICR1   REG16(0x86) // Input Capture Register (Define o Valor de Topo/Frequência)
#define _OCR1A  REG16(0x88) // Output Compare Register A (Define o Duty Cycle/Largura do Pulso)

// --- REGISTRADORES I2C (TWI) - Comunicação LCD ---
#define _TWBR   REG8(0xB8) // Bit Rate Register (Define velocidade do barramento)
#define _TWSR   REG8(0xB9) // Status Register (Prescaler do I2C)
#define _TWDR   REG8(0xBB) // Data Register (Buffer de envio/recepção)
#define _TWCR   REG8(0xBC) // Control Register (Comandos START/STOP/Escrita)

// --- REGISTRADORES UART - Telemetria para ESP32 ---
#define _UDR0   REG8(0xC6) // Data Register Serial
#define _UCSR0A REG8(0xC0) 
#define _UCSR0B REG8(0xC1) 
#define _UCSR0C REG8(0xC2) 
#define _UBRR0L REG8(0xC4) // Baud Rate Register (9600 bps)

// CONSTANTES DE TEMPO DO SERVO (Frequência de 50Hz)
#define SERVO_STOP 2600 // Pulso de ~1.5ms (Estado de repouso)
#define SERVO_DIR  2000 // Pulso de ~1.0ms (Giro para um sentido)
#define SERVO_ESQ  3000 // Pulso de ~2.0ms (Giro para sentido oposto)

// DEFINIÇÕES DE PINAGEM (Máscaras de Bits)
#define SERVO_PIN      (1 << 1)  // PB1 (Pino 9)
#define BOMBA_PIN      (1 << 5)  // PB5 (Pino 13)
#define TRIG_PIN       (1 << 7)  // PD7 (Gatilho Ultrassom)
#define ECHO_PIN       (1 << 6)  // PD6 (Retorno Ultrassom)
#define VAZAO_PIN      (1 << 2)  // PD2 (Entrada de pulsos - INT0)
#define ENC_CLK        (1 << 3)  // PD3 (Encoder Clock)
#define ENC_DT         (1 << 4)  // PD4 (Encoder Data)
#define ENC_SW         (1 << 5)  // PD5 (Botão do Encoder)

// BITS DE CONFIGURAÇÃO (Extraídos do Datasheet)
#define COM1A1  (1 << 7) // Limpar OC1A no Compare Match (PWM não-invertido)
#define WGM11   (1 << 1) // Modo Fast PWM (WGM11, 12, 13 ativos = Modo 14)
#define WGM12   (1 << 3) 
#define WGM13   (1 << 4) 
#define CS11    (1 << 1) // Prescaler de 8 para o Timer 1

#define ISC01   (1 << 1) // Interrupção INT0 na borda de descida/subida
#define ISC00   (1 << 0) 
#define INT0    (1 << 0) // Habilita interrupção externa 0
#define SREG_I  (1 << 7) // Bit de interrupção global (I-bit)

#define TWINT   (1 << 7) // Flag de interrupção I2C (Pronto para próxima tarefa)
#define TWSTA   (1 << 5) // Condição de START no I2C
#define TWSTO   (1 << 4) // Condição de STOP no I2C
#define TWEN    (1 << 2) // Habilita o hardware TWI (I2C)
#define PCF8574_ADDR   (0x27 << 1) // Endereço do expansor I2C (Shiftado para escrita)
#define LCD_BACKLIGHT  0x08        // Bit de controle da luz de fundo do LCD

// --- VARIÁVEIS GLOBAIS DE CONTROLE ---
volatile float Kp = 1.0, Ki = 0.1, Kd = 0.01;
volatile float setpoint = 15.0, vazao_atual = 0, erro = 0, nivel = 0;
float erro_anterior = 0, integral = 0;
float kpe = 0, kii = 0, kdd = 0; 

unsigned char param_sel = 0; 
volatile unsigned long pulseCount = 0; // Contador de pulsos do sensor YF-S201
unsigned char c_on = 0, c_off = 0;    // Contadores para histerese da bomba

// --- FUNÇÕES DE DELAY (Baseadas em ciclos de clock NOP) ---
void delay_us(unsigned int us) {
    while(us--) __asm__ __volatile__ ("nop"); __asm__ __volatile__ ("nop"); 
}

void delay_ms(unsigned int ms) {
    while(ms--) { 
        for(unsigned long i = 0; i < 1600; i++) __asm__ __volatile__ ("nop"); 
    }
}

// ISR (Interrupt Service Routine): Incrementa pulsos de vazão via hardware
extern "C" void __vector_1 (void) __attribute__ ((signal, used, externally_visible));
void __vector_1 (void) { pulseCount++; }

// --- UART: Inicialização e Funções de Envio ---
void uart_init() {
    _UBRR0L = 103; // Define 9600 baud rate para clock de 16MHz
    _UCSR0B = (1 << 3) | (1 << 4); // Habilita Transmissão (TX) e Recepção (RX)
    _UCSR0C = (3 << 1); // Configura 8 bits de dados e 1 bit de parada
}

void uart_putc(char c) { 
    while (!(_UCSR0A & (1 << 5))); _UDR0 = c; // Espera buffer de envio esvaziar
}

void uart_send_float(float f) {
    if (f < 0) { uart_putc('-'); f = -f; }
    int pInt = (int)f;
    int pDec = (int)((f - pInt) * 100); // Converte decimais em inteiros de 2 dígitos
    char buf[10]; int i = 0;
    if (pInt == 0) uart_putc('0');
    while (pInt > 0) { buf[i++] = (pInt % 10) + '0'; pInt /= 10; }
    while (i > 0) uart_putc(buf[--i]);
    uart_putc('.');
    if (pDec < 10) uart_putc('0');
    if (pDec == 0) uart_putc('0');
    i = 0;
    while (pDec > 0) { buf[i++] = (pDec % 10) + '0'; pDec /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

// --- I2C/LCD: Funções de baixo nível via TWI ---
void i2c_init() { 
    _TWSR = 0x00; _TWBR = 72; // Configura frequência do barramento (~100kHz)
}

void i2c_write(unsigned char data) {
    _TWDR = data; 
    _TWCR = TWINT | TWEN; // Dispara transmissão
    while (!(_TWCR & TWINT)); // Aguarda confirmação do hardware
}

void lcd_pcf(unsigned char data) {
    _TWCR = TWINT | TWSTA | TWEN; while(!(_TWCR & TWINT)); // Sinal de START
    _TWDR = PCF8574_ADDR; _TWCR = TWINT | TWEN; while(!(_TWCR & TWINT)); // Endereço do Escravo
    i2c_write(data | LCD_BACKLIGHT); // Envia dado + bit do Backlight
    _TWCR = TWINT | TWSTO | TWEN; // Sinal de STOP
}

// --- CONTROLE DE ATUAÇÃO ---
void burst_servo(int dir, float erro_mag) {
    if (erro_mag < 0) erro_mag = -erro_mag;
    unsigned char mult = 1;
    // Escalonamento do tempo de atuação proporcional à gravidade do erro
    if (erro_mag > 10.0) mult = 10;
    else if (erro_mag > 4.0) mult = 4;
    else mult = 1;

    _OCR1A = (dir == 1) ? SERVO_DIR : SERVO_ESQ;
    delay_ms(25 * mult); // O servo gira por um tempo determinado (Burst)
    _OCR1A = SERVO_STOP; // Retorna ao estado parado
}

unsigned int medir_nivel() {
    _PORTD |= TRIG_PIN; delay_us(10); _PORTD &= ~TRIG_PIN; // Pulso de trigger
    while(!(_PIND & ECHO_PIN)); // Aguarda início do eco
    unsigned int t = 0; 
    while((_PIND & ECHO_PIN) && (t < 15000)) { t++; delay_us(1); } // Mede tempo de resposta
    return t;
}

// --- LOOP PRINCIPAL ---
int main(void) {
    _DDRB  |=  SERVO_PIN | BOMBA_PIN; 
    _PORTB |=  BOMBA_PIN; // Lógica Invertida (PNP): Nível ALTO desliga a bomba
    
    _DDRD  |=  TRIG_PIN; 
    _DDRD  &=  ~(VAZAO_PIN | ENC_CLK | ENC_DT | ENC_SW | ECHO_PIN);
    _PORTD |=  VAZAO_PIN | ENC_CLK | ENC_DT | ENC_SW; // Ativa resistores de Pull-up

    // Configuração do Timer 1 para PWM de 50Hz (Período de 20ms para Servos)
    // Cálculo: Fosc / (Prescaler * Freq) => 16MHz / (8 * 50Hz) = 40.000
    _TCCR1A = COM1A1 | WGM11; 
    _TCCR1B = WGM13 | WGM12 | CS11; 
    _ICR1   = 40000; 
    _OCR1A  = SERVO_STOP;

    uart_init(); i2c_init(); 
    delay_ms(50); // Inicialização do LCD em modo 4 bits (Sequência do fabricante)
    lcd_pulse(0x30); delay_ms(5); lcd_pulse(0x30); lcd_pulse(0x20); 
    lcd_send(0x28, 0); lcd_send(0x0C, 0); lcd_send(0x01, 0); delay_ms(2);

    _EICRA = ISC01 | ISC00; _EIMSK = INT0; // Configura interrupção externa do sensor de vazão
    _SREG |= SREG_I; // Habilita interrupções globais

    unsigned long timer_geral = 0;
    unsigned char last_clk = (_PIND & ENC_CLK);
    unsigned char last_sw  = (_PIND & ENC_SW);

    atualiza_display();

    while(1) {
        timer_geral++;

        // --- LÓGICA DO ENCODER (Ajuste de Ganhos PID) ---
        unsigned char clk = (_PIND & ENC_CLK);
        if (clk != last_clk && clk == 0) {
            delay_us(500); // Debounce via software
            if ((_PIND & ENC_DT) != clk) {
                if (param_sel == 0) Kp += 0.1; else if (param_sel == 1) Ki += 0.01; else Kd += 0.01;
            } else {
                if (param_sel == 0) Kp -= 0.1; else if (param_sel == 1) Ki -= 0.01; else Kd -= 0.01;
            }
            atualiza_display();
        }
        last_clk = clk;

        // --- TAREFAS DE CONTROLE (Executadas periodicamente) ---
        if (timer_geral > 500000) {
            // 1. Segurança de Nível (Histerese de 5 ciclos)
            unsigned int d = medir_nivel();
            nivel = (float)d;
            if (d > 240) { c_on++; c_off=0; if(c_on>=5){ _PORTB &= ~BOMBA_PIN; c_on=0; } }
            else if (d < 190) { c_off++; c_on=0; if(c_off >= 5){ _PORTB |= BOMBA_PIN; c_off=0; } }

            // 2. Cálculo da Vazão (Leitura de pulsos/segundo)
            _SREG &= ~SREG_I; // Seção crítica: impede interrupção durante a leitura
            vazao_atual = (float)pulseCount; pulseCount = 0; 
            _SREG |= SREG_I;

            // 3. Algoritmo PID
            erro = setpoint - vazao_atual;
            integral += erro; 
            if(integral > 20) integral = 20; // Anti-Windup: limita acúmulo da integral

            kpe = Kp * erro; kii = Ki * integral; kdd = Kd * (erro - erro_anterior);
            float out = kpe + kii + kdd;
            erro_anterior = erro;

            // Atuação do Servo baseada no sinal do PID
            if (out > 1.0) burst_servo(1, erro); else if (out < -1.0) burst_servo(-1, erro);

            // 4. Telemetria e Interface
            uart_send_float(vazao_atual); uart_putc(','); uart_send_float(nivel); uart_putc('\n');
            atualiza_display();
            timer_geral = 0; // Reinicia ciclo de controle
        }
    }
}
