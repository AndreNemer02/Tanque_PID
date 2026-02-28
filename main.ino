/**
 * PROJETO FINAL: Controle de Tanque Baremetal 
 * FUSÃO: PID (Tiers) + Bomba/Ultrassom + LCD/Encoder + Telemetria ESP32
 */
typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;

// Macros para acesso direto à memória (ponteiros para registradores)
#define REG8(x)  (*(volatile uint8_t *)(x))
#define REG16(x) (*(volatile uint16_t *)(x))

// --- REGISTRADORES MANUAIS (AVR ATmega328P) ---
// Portas D: Controle do Trigger, Echo, Vazão e Encoder
#define _PIND   REG8(0x29)  // Leitura de entrada
#define _DDRD   REG8(0x2A)  // Configuração de Direção (0:Entrada, 1:Saída)
#define _PORTD  REG8(0x2B)  // Escrita de saída / Ativação de Pull-up
// Portas B: Controle do Servo (PWM) e Bomba
#define _PINB   REG8(0x23)
#define _DDRB   REG8(0x24)
#define _PORTB  REG8(0x25)
// Status e Interrupções Externas
#define _SREG   REG8(0x5F)  // Registrador de Status Global (Interrupções)
#define _EICRA  REG8(0x69)  // Configuração de sentido da interrupção (Borda)
#define _EIMSK  REG8(0x3D)  // Máscara de habilitação de interrupção externa

// Timer1: Configurado para PWM no Pino 9 (Servo)
#define _TCCR1A REG8(0x80)  // Configura modo de comparação e WGM (bits baixos)
#define _TCCR1B REG8(0x81)  // Configura Prescaler e WGM (bits altos)
#define _ICR1   REG16(0x86) // Valor de Topo (define a frequência do PWM)
#define _OCR1A  REG16(0x88) // Valor de Comparação (define o Duty Cycle)

// I2C (TWI - Two Wire Interface) para o LCD
#define _TWBR   REG8(0xB8)  // Bit Rate Register (Frequência do Clock)
#define _TWSR   REG8(0xB9)  // Status Register (Prescaler)
#define _TWDR   REG8(0xBB)  // Data Register (Dado a ser enviado)
#define _TWCR   REG8(0xBC)  // Control Register (Start, Stop, Enable, Interrupt)

// UART: Comunicação Serial com o ESP32
#define _UDR0   REG8(0xC6)  // Registrador de Dados
#define _UCSR0A REG8(0xC0)  // Status da transmissão/recepção
#define _UCSR0B REG8(0xC1)  // Habilita Transmissor/Receptor
#define _UCSR0C REG8(0xC2)  // Configura formato de frame (8-bit, 1 stop bit)
#define _UBRR0L REG8(0xC4)  // Baud Rate (Low byte)

// CONSTANTES DO SERVO (Valores baseados no ICR1 = 40000 para 20ms)
#define SERVO_STOP 2600 // Neutro / Parado (1.5ms)
#define SERVO_DIR  2000 // Sentido Horário (2.0ms)
#define SERVO_ESQ  3000 // Sentido Anti-horário (1.0ms)

// Definições de Máscaras de Pinos 
#define SERVO_PIN      (1 << 1)  // PB1 (Pino 9 Digital)
#define BOMBA_PIN      (1 << 5)  // PB5 (Pino 13 Digital)
#define TRIG_PIN       (1 << 7)  // PD7 (Pino 7 Digital)
#define ECHO_PIN       (1 << 6)  // PD6 (Pino 6 Digital)
#define VAZAO_PIN      (1 << 2)  // PD2 (Pino 2 Digital) - INT0
#define ENC_CLK        (1 << 3)  // PD3 (Pino 3 Digital)
#define ENC_DT         (1 << 4)  // PD4 (Pino 4 Digital)
#define ENC_SW         (1 << 5)  // PD5 (Pino 5 Digital)

// Definições de Bits de Controle (Timers e Interrupções)
#define COM1A1  (1 << 7)    // Clear OC1A on Compare Match
#define WGM11   (1 << 1)    // Fast PWM (Mode 14)
#define WGM12   (1 << 3)    // Fast PWM (Mode 14)
#define WGM13   (1 << 4)    // Fast PWM (Mode 14)
#define CS11     (1 << 1)    // Prescaler /8

#define ISC01   (1 << 1)    // Interrupção na borda de descida/subida
#define ISC00   (1 << 0)    // Qualquer mudança lógica ou borda
#define INT0     (1 << 0)    // Habilita Interrupção Externa 0
#define SREG_I  (1 << 7)    // Flag de Interrupção Global

// LCD Via PCF8574 (Expansor I2C)
#define TWINT           (1 << 7) // TWI Interrupt Flag
#define TWSTA           (1 << 5) // TWI Start Condition
#define TWSTO           (1 << 4) // TWI Stop Condition
#define TWEN            (1 << 2) // TWI Enable
#define PCF8574_ADDR    (0x27 << 1) // Endereço I2C deslocado para escrita
#define LCD_BACKLIGHT  0x08


// --- VARIÁVEIS GLOBAIS ---
// Parâmetros PID voláteis pois podem ser alterados via Encoder
volatile float Kp = 1.0, Ki = 0.1, Kd = 0.01;
volatile float setpoint = 15.0, vazao_atual = 0, erro = 0, nivel = 0;
float erro_anterior = 0, integral = 0;
float kpe = 0, kii = 0, kdd = 0; // Componentes individuais do cálculo PID

unsigned char param_sel = 0; // Menu de seleção: 0:Kp, 1:Ki, 2:Kd
volatile unsigned long pulseCount = 0; // Acumulador de pulsos do sensor de vazão
unsigned char c_on = 0, c_off = 0; // Contadores para histerese da bomba


// --- FUNÇÕES DE APOIO E DELAY ---

// Delay em microssegundos usando NOP (No Operation) para precisão em ciclos de clock
void delay_us(unsigned int us) {
    while(us--) 
        __asm__ __volatile__ ("nop"); __asm__ __volatile__ ("nop"); 
}

// Delay em milissegundos aproximado para 16MHz
void delay_ms(unsigned int ms) {
    while(ms--) { 
        for(unsigned long i = 0; i < 1600; i++) 
            __asm__ __volatile__ ("nop"); 
    }
}

// ISR (Interrupt Service Routine) do Sensor de Vazão (INT0 no Pino 2)
// Incrementa o contador de pulsos a cada transição no pino de entrada
extern "C" void __vector_1 (void) __attribute__ ((signal, used, externally_visible));
void __vector_1 (void) { pulseCount++; }


// --- UART (Comunicação ESP32 para Dashboard/Gráficos) ---
void uart_init() {
    _UBRR0L = 103; // Configura 9600 baud rate para 16MHz clock
    _UCSR0B = (1 << 3) | (1 << 4); // Habilita TX (Transmissor) e RX (Receptor)
    _UCSR0C = (3 << 1); // Configura 8 bits de dados e 1 stop bit
}

void uart_putc(char c) { 
    while (!(_UCSR0A & (1 << 5))); _UDR0 = c; // Espera buffer vazio e envia caractere
}

void uart_puts(const char* s) { 
    while (*s) uart_putc(*s++); // Envia string via UART
}

// Converte float para string e envia via serial (Telemetria)
void uart_send_float(float f) {
    if (f < 0) 
        uart_putc('-'); f = -f; 

    int pInt = (int)f;                     // Parte inteira
    int pDec = (int)((f - pInt) * 100);     // Parte decimal (2 casas)

    char buf[10]; 
    int i = 0;

    // Processamento da parte inteira
    if (pInt == 0) uart_putc('0');
    while (pInt > 0) { 
        buf[i] = (pInt % 10) + '0'; 
        pInt /= 10; 
        i++;
    }
    
    while (i > 0) uart_putc(buf[--i]);
    uart_putc('.');

    // Processamento da parte decimal
    if (pDec < 10) uart_putc('0');
    if (pDec == 0) uart_putc('0');
    while (pDec > 0) { 
        buf[i] = (pDec % 10) + '0'; 
        pDec /= 10; 
        i++;
    }

    while (i > 0) uart_putc(buf[--i]);
}


// --- I2C E LCD BAREMETAL ---

void i2c_init() { 
    _TWSR = 0x00; // Prescaler 1
    _TWBR = 72;   // SCL frequency = 100kHz para 16MHz clock
}

// Envia um byte via barramento I2C
void i2c_write(unsigned char data) {
    _TWDR = data; 
    _TWCR = TWINT | TWEN; // Inicia transmissão
    while (!(_TWCR & TWINT)); // Aguarda conclusão
}

// Envia comando/dado para o expansor PCF8574 que controla o LCD
void lcd_pcf(unsigned char data) {
    _TWCR = TWINT | TWSTA | TWEN; // Send START
    while(!(_TWCR & TWINT));

    _TWDR = PCF8574_ADDR; // Send Slave Address
    _TWCR = TWINT | TWEN; 
    while(!(_TWCR & TWINT));

    i2c_write(data | LCD_BACKLIGHT); // Envia dado com luz de fundo ligada
    _TWCR = TWINT | TWSTO | TWEN; // Send STOP
}

// Gera o pulso de Enable (E) necessário para o LCD processar os dados
void lcd_pulse(unsigned char d) {
    lcd_pcf(d | 0x04); // Enable HIGH
    delay_us(1);
    lcd_pcf(d & ~0x04); // Enable LOW
    delay_us(50);
}

// Envia um byte dividindo em dois nibbles (Modo 4 bits)
void lcd_send(unsigned char val, unsigned char rs) {
    lcd_pulse((val & 0xF0) | rs); // Nibble superior
    lcd_pulse(((val << 4) & 0xF0) | rs); // Nibble inferior
}

void lcd_puts(const char* s) {
    while (*s) {
        lcd_send(*s++, 1); // RS=1 para envio de caracteres (Data)
    }
}

// Função auxiliar para conversão de inteiro para string (sem bibliotecas padrão)
void itoa_simple(int n, char* s) {
    int i = 0, sign = n; 
    if (n < 0) n = -n;

    do {
         s[i++] = n % 10 + '0'; 
    } while ((n /= 10) > 0);

    if (sign < 0) s[i++] = '-'; 
    s[i] = '\0';

    // Inverte a string gerada
    for (int j = 0, k = i - 1; j < k; j++, k--) { 
        char t = s[j]; 
        s[j] = s[k]; 
        s[k] = t; 
    }
}

// Formata e imprime um float no LCD com 2 casas decimais
void print_float_lcd(float val) {
    char b[10]; 
    int ip = (int)val, dp = (int)((val - ip) * 100);

    if (dp < 0) 
        dp = -dp; itoa_simple(ip, b);

    char *p = b; 
    while(*p)
        lcd_send(*p++, 1);

    lcd_send('.', 1); 
    if (dp < 10) 
        lcd_send('0', 1);

    itoa_simple(dp, b);

    p = b; 
    while(*p) 
        lcd_send(*p++, 1);
}

// Gerencia as informações exibidas no LCD
void atualiza_display() {
    lcd_send(0x80, 0); // Posiciona cursor na Linha 1
    lcd_puts("E:"); 
    print_float_lcd(erro);
    lcd_send(0xC0, 0); // Posiciona cursor na Linha 2

    if (param_sel == 0)      { lcd_puts("Kp:"); print_float_lcd(Kp); }
    else if (param_sel == 1) { lcd_puts("Ki:"); print_float_lcd(Ki); }
    else                     { lcd_puts("Kd:"); print_float_lcd(Kd); }
}

// --- ATUAÇÃO E SENSORES ---
// Função que move o servo proporcionalmente à magnitude do erro (Burst Control)
void burst_servo(int dir, float erro_mag) {
    if (erro_mag < 0) erro_mag = -erro_mag;

    unsigned char mult = 1;
    // Escalonamento da duração do movimento conforme o erro
    if (erro_mag > 10.0) mult = 10;
    else if (erro_mag > 8.0) mult = 8;
    else if (erro_mag > 6.0) mult = 6;
    else if (erro_mag > 4.0) mult = 4;
    else if (erro_mag > 2.0) mult = 2;
    else mult = 1;

    _OCR1A = (dir == 1) ? SERVO_DIR : SERVO_ESQ; // Define direção do pulso
    delay_ms(25 * mult); // Duração do movimento proporcional ao erro
    _OCR1A = SERVO_STOP; // Para o servo
}

// Função de leitura do sensor ultrassônico HC-SR04 via Baremetal
unsigned int medir_nivel() {
    _PORTD |= TRIG_PIN; // Pulso de Trigger HIGH
    delay_us(10); 
    _PORTD &= ~TRIG_PIN; // Trigger LOW

    while(!(_PIND & ECHO_PIN)); // Aguarda o início do pulso de retorno (Echo)

    unsigned int t = 0; 
    // Conta o tempo enquanto Echo está HIGH ou até estourar o timeout
    while((_PIND & ECHO_PIN) && (t < 15000)) 
    { 
        t++; delay_us(1); 
    }

    return t; // Retorna valor proporcional à distância
}

// --- MAIN ---
int main(void) {

    // Configura PB1 (Servo) e PB5 (Bomba) como saída
    _DDRB  |=  SERVO_PIN | BOMBA_PIN; 
    _PORTB |=  BOMBA_PIN;   // Estado inicial: Desliga a bomba  
    
    // Configura PD7 (Trigger do Ultrassom) como Saída
    _DDRD  |=  TRIG_PIN;     
    
    // Configura PD2 a PD6 como Entradas (Vazão, Encoder e Echo)
    _DDRD  &=  ~(VAZAO_PIN | ENC_CLK | ENC_DT | ENC_SW | ECHO_PIN);

    // Ativa Resistores de Pull-up internos para garantir nível lógico estável
    _PORTD |=  VAZAO_PIN | ENC_CLK | ENC_DT | ENC_SW;

    // Configuração do Timer1: Fast PWM, Topo em ICR1, Prescaler /8
    _TCCR1A = COM1A1 | WGM11; 
    _TCCR1B = WGM13 | WGM12 | CS11; 
    _ICR1   = 40000;  // Período de 20 ms (50Hz) para servos
    _OCR1A  = SERVO_STOP; // Inicia em posição neutra

    uart_init(); 
    i2c_init(); 
    
    // Sequência de Inicialização do LCD em modo 4 bits (Protocolo HD44780)
    delay_ms(50); lcd_pulse(0x30); delay_ms(5); lcd_pulse(0x30); delay_us(200);
    lcd_pulse(0x30); delay_us(200); lcd_pulse(0x20); lcd_send(0x28, 0); 
    lcd_send(0x0C, 0); lcd_send(0x01, 0); delay_ms(2);

    // Configuração de Interrupções Externas (Sensor de Vazão)
    _EICRA = ISC01 | ISC00; // Configura borda de subida para INT0
    _EIMSK = INT0;          // Habilita interrupção INT0
    _SREG |= SREG_I;        // Habilita interrupções globais

    unsigned long timer_geral = 0;
    unsigned char last_clk    = (_PIND & ENC_CLK);
    unsigned char last_sw      = (_PIND & ENC_SW);

    atualiza_display();

    while(1) {
        timer_geral++;

        // --- LÓGICA DO ENCODER ROTATIVO (Polling) ---
        unsigned char clk = (_PIND & ENC_CLK);
        if (clk != last_clk && clk == 0) { // Detecção de giro (borda de descida no CLK)
            delay_us(500); // Anti-debounce
            if ((_PIND & ENC_DT) != clk) { // Sentido Horário
                if (param_sel == 0) Kp += 0.1; 
                else if (param_sel == 1) Ki += 0.01; 
                else Kd += 0.01;
            } else { // Sentido Anti-horário
                if (param_sel == 0) Kp -= 0.1; 
                else if (param_sel == 1) Ki -= 0.01; 
                else Kd -= 0.01;
            }
            // Impede valores negativos nos parâmetros do controle
            if (Kp < 0) Kp = 0; 
            if (Ki < 0) Ki = 0; 
            if (Kd < 0) Kd = 0;
            atualiza_display();
        }
        last_clk = clk;

        // Botão do Encoder (Troca de parâmetro Kp/Ki/Kd)
        unsigned char sw = (_PIND & ENC_SW);
        if (!sw && last_sw) {
            delay_ms(20); 
            param_sel = (param_sel + 1) % 3; 
            atualiza_display();
        }
        last_sw = sw;

        // --- TAREFAS PERIÓDICAS (Loop de Controle Principal) ---
        if (timer_geral > 500000) {

            // 1. Controle de nível da bomba (Histerese/Bang-bang)
            unsigned int d = medir_nivel();
            nivel = (float)d;
            if (d > 240) { // Nível muito baixo (distância grande)
                c_on++; 
                c_off=0; 
                if(c_on>=5){ 
                    _PORTB &= ~BOMBA_PIN; // Liga bomba (Lógica invertida/Ativo baixo)
                    c_on=0; 
                } 
            }
            else if (d < 190) { // Nível muito alto (distância curta)
                c_off++;
                c_on=0; 
                if(c_off >= 5 ) { 
                    _PORTB |= BOMBA_PIN; // Desliga bomba
                     c_off=0; 
                } 
            }

            // 2. Cálculo da vazão atual (Seção Crítica: desabilita interrupção para ler pulseCount)
            _SREG &= ~SREG_I; 
            vazao_atual = (float)pulseCount;
            pulseCount = 0; 
            _SREG |= SREG_I;

            // 3. Algoritmo de Controle PID (Tiers)
            erro = setpoint - vazao_atual;
            integral += erro; 
            if(integral > 20) integral = 20; // Anti-windup (limita acumulo da integral)

            kpe = Kp * erro; 
            kii = Ki * integral; 
            kdd = Kd * (erro - erro_anterior);
            
            float out = kpe + kii + kdd; // Saída combinada PID
            erro_anterior = erro;

            // Atuação discreta no servo baseada na zona morta (Deadband) de 1.0/-1.0
            if (out > 1.0) 
                burst_servo(1, erro);
            else if (out < -1.0) burst_servo(-1, erro);

            // 4. Telemetria: Envio de dados via UART formatados para CSV (ESP32)
            uart_send_float(vazao_atual); uart_putc(',');
            uart_send_float(nivel);       uart_putc(',');
            uart_send_float(erro);        uart_putc(',');
            uart_send_float(kpe);         uart_putc(',');
            uart_send_float(kii);         uart_putc(',');
            uart_send_float(kdd);         uart_putc('\n');

            atualiza_display();
            timer_geral = 0; // Reseta temporizador do loop de controle
        }
    }
}
