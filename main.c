#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h" //biblioteca para controlar o hardware de PWM

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22 // GPIO para botão do Joystick
#define BOTAO_A 5 // GPIO para botão A

#define LED_R 13
#define LED_G 11
#define LED_B 12

//Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define BOTAO_B 6

const uint16_t WRAP_PERIOD = 4000; //valor máximo do contador - WRAP
const float PWM_DIVISER = 4.0; //divisor do clock para o PWM
uint16_t led_level_b = 0; //nível inicial do pwm do led blue (duty cycle)
uint16_t led_level_r = 0; //nível inicial do pwm do led red(duty cycle)

int bord_type = 0;
bool ligar_pwm = false;

//função para configurar o módulo PWM
void pwm_setup()
{
    gpio_set_function(LED_R, GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM
    gpio_set_function(LED_B, GPIO_FUNC_PWM); 

    uint slice = pwm_gpio_to_slice_num(LED_R); //obter o canal PWM da GPIO
    uint slice1 = pwm_gpio_to_slice_num(LED_B); 

    pwm_set_clkdiv(slice, PWM_DIVISER); //define o divisor de clock do PWM
    pwm_set_clkdiv(slice1, PWM_DIVISER); 

    pwm_set_wrap(slice, WRAP_PERIOD); //definir o valor de wrap
    pwm_set_wrap(slice1, WRAP_PERIOD); 

    pwm_set_gpio_level(LED_R, 0); //definir o cico de trabalho (duty cycle) do pwm
    pwm_set_gpio_level(LED_B, 0); 

    pwm_set_enabled(slice, true); //habilita o pwm no slice correspondente
    pwm_set_enabled(slice1, true); 
}

void gpio_irq_handler(uint gpio, uint32_t events)
{  
  //debouncing
  static uint32_t last_time = 0;
  uint32_t current_time = to_ms_since_boot(get_absolute_time());
  if (current_time - last_time < 200) {
      return;
  }
  last_time = current_time;

  if (gpio == BOTAO_B){
    reset_usb_boot(0, 0);
  } else if (gpio == BOTAO_A){    
    ligar_pwm = !ligar_pwm;
    pwm_set_gpio_level(LED_R, 0);
    pwm_set_gpio_level(LED_B, 0);    
  } else if (gpio == JOYSTICK_PB){
    gpio_put(LED_G,!gpio_get(LED_G));
    bord_type += 1;
    if (bord_type >= 3)
      bord_type = 0;
  }
  
}

void ajuste_nivel_led(uint16_t adc_value, uint gpio) {
  int brilho = 0;

  //pequena margem de valores para reduzir a oscilaçao e centralizar o joystick
  if (adc_value > 1900 && adc_value < 2050  ) {
    adc_value = 2048;
  }  

  // Se estiver no centro apaga o LED
  if (adc_value == 2048) {
      brilho = 0;
  } else {
      // Calcula o brilho proporcional ao afastamento do centro
      brilho = abs(adc_value - 2048);
  }
  
  if (ligar_pwm) {
      if (gpio == LED_B) {
          led_level_b = brilho;
          pwm_set_gpio_level(LED_B, led_level_b);
      } else if (gpio == LED_R) {
          led_level_r = brilho;
          pwm_set_gpio_level(LED_R, led_level_r);
      }
  }
} 

int main()
{

  stdio_init_all(); //inicializa o sistema padrão de I/O
  pwm_setup(); //configura o PWM

  gpio_init(LED_G);
  gpio_set_dir(LED_G,GPIO_OUT);

  // Para ser utilizado o modo BOOTSEL com botão B
  gpio_init(BOTAO_B);
  gpio_set_dir(BOTAO_B, GPIO_IN);
  gpio_pull_up(BOTAO_B);

  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB); 

  gpio_init(BOTAO_A);
  gpio_set_dir(BOTAO_A, GPIO_IN);
  gpio_pull_up(BOTAO_A);

  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN); 

  uint16_t adc_value_x;
  uint16_t adc_value_y;    

  bool cor = true;
  while (true)
  {
    adc_select_input(0); // Seleciona o ADC para eixo X. O pino 26 como entrada analógica
    adc_value_x = adc_read();
    adc_select_input(1); // Seleciona o ADC para eixo Y. O pino 27 como entrada analógica
    adc_value_y = adc_read();           
    
    // Converte os valores do ADC para coordenadas do ecrã corretamente
    int pos_x = 56 - ((adc_value_x * 56) / 4095); // Mapeia X para [0,120]
    int pos_y = (adc_value_y * 120) / 4095;  // Mapeia Y para [0,56]

    // pos_x += 4; // Ajusta para ficar dentro da área útil (128 px)
    // pos_y += 4; // Ajusta para ficar dentro da área útil (64 px)
    
    // Atualiza o conteúdo do display com animações
    ssd1306_fill(&ssd, !cor); // Limpa o display

    //escolha dos tipos de borda
    switch (bord_type)
    {
        case 0:
            ssd1306_rect(&ssd, 3, 3, 122, 61, cor, !cor); // Desenha um retângulo
            break;
        case 1:
            ssd1306_line(&ssd, 3, 3, 122, 3, cor);   // Linha no topo
            ssd1306_line(&ssd, 3, 62, 122, 62, cor); // Linha na parte inferior
            break;
        case 2:
            ssd1306_line(&ssd, 3, 3, 3, 61, cor);   // Linha na esquerda
            ssd1306_line(&ssd, 122, 3, 122, 61, cor); // Linha na direita
            break;
    }   
    
    //Desenha o quadrado do joytick
    ssd1306_rect(&ssd, pos_x, pos_y, 8, 8, cor, 1);   
    
    if (ligar_pwm){
      ajuste_nivel_led(adc_value_x,LED_R); 
      ajuste_nivel_led(adc_value_y,LED_B);
    }    
         
    ssd1306_send_data(&ssd); // Atualiza o display
    sleep_ms(100);
  }
}