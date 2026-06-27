// Autores: Carlos Eduardo Alves Silva Lira e João Victor Rodrigues dos Santos 
// Projeto Final - Arduino UNO 
// Acionamento de bomba hidráulica a partir do monitoramento do reservatório e caixa d'água.
// A bomba é disparada por intermédio de um relé, 3 boias de nível para aferição do reservatório,
// Sensor ultrassônico e uma boia de nível no topo para a caixa d'água.

// Componentes
/*
Display HD44780 (Endereço I2C 0x27 PCF8574)
Sensor ultrassônico HC-SR04
Sensores de nível tipo boia (interrupção magnética)
Push-buttons
Relé HW-482
*/

// macros

#define SBIT(N) (1<<(N)) 
#define I2C_ADDR 0x27 // endereço I2C do display
#define ALTURA_CAIXA 200 // 2 metros (200 cm)

// variáveis globais iniciadas
volatile byte modo = 0; // inicia em 0 (MANUAL) por segurança
volatile byte display_ligado = 1; 
volatile byte bomba_manual = 0; 

// variáveis do ultrassônico
volatile byte descida = 0;
volatile byte nova_medicao = 0;
volatile unsigned int tempoSubida;
volatile unsigned int tempoDescida;

// temporização de 5s e 50ms
volatile unsigned int cont_1ms = 0;
volatile byte flag_5s = 1;
volatile byte cont_50ms = 0; // contador para atualização do display
volatile byte flag_50ms = 0; // flag para liberação de atualização do display

// estados de nível
byte c_pct = 0; 
byte r_pct = 0; 
byte r_bloqueado = 0; 

// UART
void uart_init() {
    UBRR0 = 16; // 115200 baud para 16MHz  
    UCSR0A = (1 << U2X0);// ativa velocidade dupla de transmissão
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // habilitando recebimento, envio e interrupção de dados 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits de dados sem paridade
}

void uart_tx(char c) {
    while (!(UCSR0A & (1 << UDRE0))); 
    UDR0 = c; // coloca o caractere no registrador de dados para envio
}

void uart_print(const char *str) {
    // envia a string caractere por caractere
    while (*str) uart_tx(*str++);
}

void uart_print_num(byte num) {
    // converte o número (0 a 100) para caracteres ASCII manualmente para envio via serial
    if (num == 100) { uart_print("100"); }
    else if (num >= 10) { uart_tx('0' + (num/10)); uart_tx('0' + (num%10)); }
    else { uart_tx('0' + num); }
}

// I2C
void i2c_init() { 
    TWSR = 0; // prescaler do I2C em 1
    TWBR = 72; // gera uma frequência SCL de 100kHz para um clock de 16MHz (Modo Standard I2C)
    TWCR = (1<<TWEN); // habilita o I2C
}
void i2c_start() { 
    // gera a condição de START e aguarda a confirmação
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while(!(TWCR & (1<<TWINT))); 
}
void i2c_stop() { 
    // gera a condição de STOP
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); 
}
void i2c_write(byte data) { 
    // envia 1 byte de dado pelo barramento e aguarda a confirmação de conclusão (TWINT)
    TWDR = data; TWCR = (1<<TWINT)|(1<<TWEN); while(!(TWCR & (1<<TWINT))); 
}

void lcd_send_nibble(byte nibble, byte is_data) {
    // prepara os 4 bits.
    // 0x08 = Liga luz de fundo | 0x00 = Desliga luz de fundo
    byte control = (display_ligado ? 0x08 : 0x00) | is_data; 
    byte payload = (nibble & 0xF0) | control;
    i2c_start(); 
    i2c_write(I2C_ADDR << 1); // envia o endereço do escravo deslocado 1 bit (padrão I2C)
    
    // gera o pulso no pino ENABLE (0x04) do display para que ele grave a informação
    i2c_write(payload | 0x04); _delay_us(1); 
    i2c_write(payload & ~0x04); _delay_us(50);
    i2c_stop();
}

void lcd_send_byte(byte valor, byte is_data) {
    // divide o byte inteiro em dois pedaços de 4 bits e os envia em sequência
    lcd_send_nibble(valor & 0xF0, is_data); 
    lcd_send_nibble((valor << 4) & 0xF0, is_data);
}

void lcd_comando(byte cmd) { lcd_send_byte(cmd, 0); } // 0 = Modo de instrução/comando (RS low)
void lcd_caractere(char c) { lcd_send_byte(c, 1); }   // 1 = Modo de escrita de dado (RS high)
void lcd_print(const char *str) { while (*str) lcd_caractere(*str++); }

void lcd_print_num(byte num) {
    if (num == 100) { lcd_print("100"); }
    else if (num >= 10) { lcd_caractere('0' + (num/10)); lcd_caractere('0' + (num%10)); }
    else { lcd_caractere('0' + num); }
}

void lcd_init_i2c() {
    // sequência obrigatória de inicialização do controlador HD44780
    _delay_ms(50);
    lcd_send_nibble(0x30, 0); _delay_ms(5);
    lcd_send_nibble(0x30, 0); _delay_us(150);
    lcd_send_nibble(0x30, 0); lcd_send_nibble(0x20, 0); // Define interface para 4 bits
    lcd_comando(0x28); lcd_comando(0x0C); lcd_comando(0x01); _delay_ms(2);
}

// configuração de hardware e timers
void confPin() {
    // DDRx configura a direção: 1 = Saída, 0 = Entrada
    DDRB |= SBIT(DDB5); // Pino 13 (PB5) como saída (usado para o pino Trigger do sensor)
    
    // Pino 2 (PD2) como entrada (usado para o pino Echo do sensor, atrelado ao INT0)
    DDRD &= ~(1<<DDD2); 
    // pull-up desligado.
    PORTD &= ~(1<<PORTD2); 
    
    // Pino 8 (PB0) configurado como saída para acionar o relé da bomba
    DDRB |= (1<<PB0);
    
    // configura PC0, PC1, PC2 (boias do reservatório) e PC3 (boia da caixa) como entradas
    DDRC &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)); 
    // pull-up ativado
    PORTC |= ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3));
    
    // pinos dos botões
    DDRD &= ~((1<<PD4)|(1<<PD3)); 
    PORTD |= ((1<<PD4)|(1<<PD3));
}

void confInt0() { 
    // configura a interrupção externa 0 (INT0) no pino PD2
    // ISC01 e ISC00 = 11: Configura para disparar na borda de SUBIDA do sinal
    EICRA |= (1<<ISC01) | (1<<ISC00);
    EIMSK |= (1<<INT0); // habilita a interrupção
}

void confTimer1() {
    // O Timer 1 é de 16 bits. Será usado para automatizar a leitura do sensor HC-SR04
    TCCR1A = 0; // limpando
    TCCR1B = 0; 
    
    // WGM12: Modo CTC (O timer zera quando atinge o valor máximo OCR1A)
    // CS11 e CS10: Define prescaler para 64 (Com 16MHz, cada pulso leva 4 microssegundos)
    TCCR1B |= SBIT(WGM12) | SBIT(CS11) | SBIT(CS10); 
    
    // define quando o timer deve zerar e disparar a interrupção A.
    // 14999 pulsos * 4us = aprox 60 milissegundos. Taxa para evitar ecos no HC-SR04.
    OCR1A = 14999;
    
    // Define a interrupção B. (2 * 4us = 8 microssegundos)
    // cria um pulso para Trigger.
    OCR1B = 2; 
    TCNT1 = 0; // Zera a contagem atual do timer
    
    // Habilita as duas interrupções de comparação (A e B)
    TIMSK1 |= SBIT(OCIE1A); 
    TIMSK1 |= SBIT(OCIE1B); 
}

void confTimer2() {
    // timer (1ms) para o tratamento debounce e outras flags de tempo
    TCCR2A = (1<<WGM21); // Modo CTC
    OCR2A = 249;         // Com prescaler de 64, (16MHz/64)/(249+1) = 1000 Hz (1ms)
    TCCR2B = (1<<CS22);  // Define prescaler 64
    TIMSK2 = (1<<OCIE2A); // Habilita interrupção por comparação A
}

// interrupções
ISR(TIMER1_COMPA_vect) { 
    // Executada a cada 60ms. Liga o pino Trigger (Inicia o pulso ultrassônico)
    PORTB |= SBIT(PORTB5); 
}

ISR(TIMER1_COMPB_vect) { 
    // Executada 8us após a COMPA. Desliga o pino Trigger (Finaliza o pulso)
    PORTB &= ~SBIT(PORTB5); 
}

ISR(INT0_vect) { 
    // Executa quando há mudança de estado no pino Echo (recebimento do eco ultrassônico)
    if (descida == 0) { 
        // Acabou de ler a borda de subida (Início do sinal Echo)
        tempoSubida = TCNT1; // Salva o tempo atual do timer
        descida = 1; 
        EICRA &= ~(1<<ISC00); // Muda o gatilho da interrupção para borda de DESCIDA (ISC = 10)
    } 
    else { 
        // Acabou de ler a borda de descida (Fim do sinal Echo)
        tempoDescida = TCNT1; // Salva o tempo final
        descida = 0; 
        EICRA |= (1<<ISC00); // Volta o gatilho para borda de SUBIDA (ISC = 11) para a próxima leitura
        nova_medicao = 1; // Sinaliza ao loop() que temos uma medida pronta
    } 
}

ISR(USART_RX_vect) {
    // Executada sempre que um novo caractere chega pela porta Serial
    char c = UDR0; // Lê o caractere do registrador de dados da UART
    
    // verifica o estado da boia superior de segurança diretamente pelo registrador (PINC)
    byte boia_caixa_alta = (PINC & (1<<PC3)); 
    
    switch (c) {
        case 'D':
        case 'd':
            // Comando 'D' ou 'd': Retorna os dados (Data)
            uart_print("C:");
            uart_print_num(c_pct); 
            uart_print("% R:"); 
            uart_print_num(r_pct); 
            uart_print("% MODO: ");
            if (modo == 0) uart_print("MANUAL\r\n");
            else if (modo == 1) uart_print("AUTO M\r\n");
            else if (modo == 2) uart_print("AUTO B\r\n");
            break;
            
        case 'M':
        case 'm':
            // Comando 'M' ou 'm': Alterna o Modo de operação ciclicamente
            modo = (modo == 2) ? 0 : modo + 1;
            uart_print("OK. MODO: ");
            if (modo == 0) uart_print("MANUAL\r\n");
            else if (modo == 1) uart_print("AUTO M\r\n");
            else if (modo == 2) uart_print("AUTO B\r\n");
            break;
            
        case 'L':
        case 'l':
            // Comando 'L' ou 'l': Liga a bomba manualmente, respeitando o intertravamento
            if (r_pct <= 25) {
                uart_print("ERRO. R: BAIXO\r\n");
            } 
            else if (c_pct >= 90 || boia_caixa_alta) { 
                uart_print("ERRO. C: ALTO \r\n");
            }
            else {
                modo = 0; // Força modo manual
                bomba_manual = 1;
                uart_print("BOMBA LIGADA\r\n");
            }
            break;
            
        case 'P':
        case 'p':
            // Comando 'P' ou 'p': Para a bomba (Pausa)
            modo = 0;
            bomba_manual = 0;
            uart_print("BOMBA PARADA\r\n");
            break;
            
        default:
            // Ignora outros caracteres
            break;
    }
}

ISR(TIMER2_COMPA_vect) {
    // Executada exatamente a cada 1 milissegundo. Usada para tempo e filtro de botões.
    static byte deb_btn1 = 0, deb_btn2 = 0;
    static byte btn1_ativo = 0, btn2_ativo = 0;

    // só aceita o botão se ele ficar em estado baixo por 30ms
    if (!(PIND & (1<<PD4))) {
        if (deb_btn1 < 30) deb_btn1++;
        else if (deb_btn1 == 30 && !btn1_ativo) {
            btn1_ativo = 1;
            deb_btn1++;
            modo = (modo == 2) ? 0 : modo + 1;
        }
    } else { deb_btn1 = 0; btn1_ativo = 0; } // reseta a contagem se o botão soltar no meio do tempo

    if (!(PIND & (1<<PD3))) {
        if (deb_btn2 < 30) deb_btn2++;
        else if (deb_btn2 == 30 && !btn2_ativo) {
            btn2_ativo = 1;
            deb_btn2++;
            display_ligado = !display_ligado; // liga ou desliga a luz de fundo do display
            lcd_comando(display_ligado ? 0x0C : 0x08);
        }
    } else { deb_btn2 = 0; btn2_ativo = 0; }

    // atinge a flag a cada 5000 ms (5 segundos).
    cont_1ms++;
    if (cont_1ms >= 5000) { cont_1ms = 0; flag_5s = 1; }
    
    // base de tempo curta para o display (50 milissegundos).
    cont_50ms++;
    if (cont_50ms >= 50) { cont_50ms = 0; flag_50ms = 1; }
}

// lógica de controle da bomba 
void controla_bomba() {
    // lê o estado direto do pino da boia da caixa (Proteção contra transbordo caso o ultrassônico falhe)
    byte boia_caixa_alta = (PINC & (1<<PC3));
    
    // Bloqueia imediatamente o funcionamento em caso de reservatório seco ou caixa cheia
    if (r_bloqueado || c_pct >= 90 || boia_caixa_alta) { 
        PORTB &= ~(1<<PB0); // Desliga o Relé (Bomba parada)
        bomba_manual = 0; // Zera a flag manual de segurança para não religar sozinho ao sair da condição
        return;
    }

    if (modo == 0) { // modo manual
        // Controlado via Serial ou botões indiretos
        if (bomba_manual) PORTB |= (1<<PB0);
        else PORTB &= ~(1<<PB0);
    } 
    else if (modo == 1) { // modo automático médio
        // Liga a bomba se o nível baixar da metade
        if (c_pct <= 50) PORTB |= (1<<PB0);
    } 
    else if (modo == 2) { // modo automático baixo
        // Liga a bomba apenas quando estiver perto de acabar a água
        if (c_pct <= 25) PORTB |= (1<<PB0);
    }
}

void setup() {

    confPin();
    i2c_init();
    uart_init();
    lcd_init_i2c();
    confTimer1();
    confTimer2();
    confInt0();
    
    uart_print("SISTEMA INICIADO\r\n"); 
    sei(); // habilita as interrupções globais
}

void loop() {
    // roda a cada 5 segundos para cálculos e atualizações de sensores
    if (flag_5s) {
        flag_5s = 0; // zera a flag para aguardar o próximo ciclo
        
        // cálculo do ultrassônico
        if (nova_medicao == 1) { 
            unsigned int tempoEcho, tempo_microssegundos, distancia_cm;
            
            // cli() desabilita temporariamente as interrupções
            cli(); 
            unsigned int copia_subida = tempoSubida;
            unsigned int copia_descida = tempoDescida;
            sei(); // reabilita interrupções

            // como o timer zera a cada 60ms, o tempo de descida pode ser menor que o de subida se a medição cruzou o limite do timer.
            if (copia_descida >= copia_subida) tempoEcho = copia_descida - copia_subida;
            else tempoEcho = 15000 - copia_subida + copia_descida; // 15000 (14999 + 1) é o valor máximo do Timer1

            // Multiplica por 4 pois nosso prescaler (64) faz 4 microssegundos.
            tempo_microssegundos = tempoEcho * 4;
            // Divisão matemática padrão (Velocidade do som) para transformar tempo(us) ida/volta em distância(cm).
            distancia_cm = tempo_microssegundos / 58;
            
            // Limita leitura para o cálculo da porcentagem não ficar negativo
            if (distancia_cm > ALTURA_CAIXA) distancia_cm = ALTURA_CAIXA;
            
            // Calcula o volume inverso da caixa (quanto menor a distância pro teto, mais cheia ela está)
            c_pct = ((ALTURA_CAIXA - distancia_cm) * 100) / ALTURA_CAIXA;
            nova_medicao = 0; 
        }

        // Leitura lógica rápida do estado dos pinos do reservatório
        // retorna 1 (boia submersa) ou 0 (seca).
        byte s1 = (PINC & (1<<PC0)) != 0;
        byte s2 = (PINC & (1<<PC1)) != 0;
        byte s3 = (PINC & (1<<PC2)) != 0;
        byte qtd = s1 + s2 + s3;

        // Três boias de contato simples traduzidas em porcentagem
        if (qtd == 3) r_pct = 75;
        else if (qtd == 2) r_pct = 50;
        else if (qtd == 1) r_pct = 25;
        else r_pct = 0;

        // Bloqueio. Se zerar, bloqueia. Só desbloqueia se chegar a 50% de novo
        if (r_pct == 0) r_bloqueado = 1; 
        else if (r_pct >= 50) r_bloqueado = 0; 
    }

    // máquina de estados da bomba
    controla_bomba();
    
    //atualização do display
    if (flag_50ms) {
        flag_50ms = 0; // zera a flag para aguardar os próximos 50ms
        
        if (display_ligado) {
            lcd_comando(0x80); // move o cursor do LCD para a linha 1, coluna 1
            lcd_print("C:"); lcd_print_num(c_pct);
            lcd_print("% R:"); lcd_print_num(r_pct); lcd_print("%  ");

            lcd_comando(0xC0); // move o cursor do LCD para a linha 2, coluna 1
            if (modo == 0) lcd_print("MODO: MANUAL   ");
            else if (modo == 1) lcd_print("MODO: AUTO M   ");
            else if (modo == 2) lcd_print("MODO: AUTO B   ");
        }
    }
}